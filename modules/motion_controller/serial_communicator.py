"""
Enhanced Serial communicator for interfacing with the Arduino.
Features improved reliability, auto-reconnection, and command retries.
"""
import serial
import time
import logging
import threading
import queue
from datetime import datetime

logger = logging.getLogger(__name__)

class SerialCommunicator:
    """
    Manages serial communication with the Arduino.
    Handles command sending and response parsing with improved reliability.
    """
    
    def __init__(self, port, event_bus, baudrate=9600, timeout=1.0, demo_mode=False):
        """
        Initialize the serial communicator.
        
        Args:
            port: Serial port name
            event_bus: EventBus for inter-module communication
            baudrate: Serial baud rate
            timeout: Serial timeout in seconds
            demo_mode: Whether to run in demo mode without real hardware
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.event_bus = event_bus
        self.demo_mode = demo_mode
        self.ser = None
        self.connected = False
        self.last_command = None
        self.last_response = None
        self.last_command_time = None
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.last_connection_attempt = 0
        self.reconnect_cooldown = 5  # Seconds between reconnection attempts
        
        # Command retry settings
        self.max_retries = 3
        self.retry_delay = 0.5
        
        # Command processing delays
        self.pre_command_delay = 0.1  # Delay before sending a command
        self.post_command_delay = 0.2  # Delay after sending a command
        
        self.send_queue = queue.Queue()
        self.response_queue = queue.Queue()
        
        self.reader_thread = None
        self.writer_thread = None
        self.watchdog_thread = None
        self.running = False
        
        # Create a lock for thread safety
        self.serial_lock = threading.Lock()
        
        # Initialize serial connection
        self._connect()
        
        # Register event handlers
        self.event_bus.register('serial_command', self.handle_command)
        
        # Start threads if connected
        if self.connected:
            self._start_threads()
    
    def _connect(self):
        """Initialize the serial connection to the Arduino with robust error handling."""
        if self.demo_mode:
            logger.info("Running in DEMO mode - no serial connection will be established")
            self.connected = False
            return
        
        # Check reconnection cooldown
        current_time = time.time()
        if current_time - self.last_connection_attempt < self.reconnect_cooldown:
            logger.debug("Waiting for reconnection cooldown")
            return False
        
        self.last_connection_attempt = current_time
        
        # Reconnection attempt limit
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            logger.warning(f"Max reconnection attempts ({self.max_reconnect_attempts}) reached")
            # Reset counter to allow future attempts
            self.reconnect_attempts = 0
            return False
        
        try:
            # If serial port is already open, close it first
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    logger.debug("Closed existing serial connection before reconnecting")
                except Exception as e:
                    logger.warning(f"Error closing existing serial connection: {e}")
            
            # Open new connection
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            logger.info(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
            
            # Important: Allow longer time for Arduino to reset and stabilize
            time.sleep(3)
            
            # Clear any pending data
            if self.ser.in_waiting:
                self.ser.reset_input_buffer()
            
            # Test the connection with a ping
            self.ser.write(b"O\n")  # Simple ping
            time.sleep(0.5)
            
            # Read response if available
            response = ""
            if self.ser.in_waiting:
                response = self.ser.readline().decode().strip()
            
            # Reset reconnection counter on success
            self.reconnect_attempts = 0
            self.connected = True
            
            return True
            
        except Exception as e:
            self.reconnect_attempts += 1
            logger.error(f"Failed to connect to Arduino (attempt {self.reconnect_attempts}): {e}")
            self.ser = None
            self.connected = False
            return False
    
    def _start_threads(self):
        """Start the reader, writer and watchdog threads."""
        self.running = True
        
        # Start reader thread
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        
        # Start writer thread
        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.writer_thread.start()
        
        # Start watchdog thread to monitor connection
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()
        
        logger.debug("Serial communication threads started")
    
    def _watchdog_loop(self):
        """Background thread to monitor serial connection health and reconnect if needed."""
        WATCHDOG_INTERVAL = 10  # Check connection every 10 seconds
        
        while self.running:
            # Only run watchdog if not in demo mode
            if not self.demo_mode:
                if not self.connected or self.ser is None or not self.ser.is_open:
                    logger.warning("Watchdog detected disconnected state, attempting to reconnect")
                    if self._connect():
                        # If reconnection successful, restart communication threads
                        if self.reader_thread is None or not self.reader_thread.is_alive():
                            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
                            self.reader_thread.start()
                        
                        if self.writer_thread is None or not self.writer_thread.is_alive():
                            self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
                            self.writer_thread.start()
                        
                        logger.info("Successfully reconnected and restarted threads")
                else:
                    # Test the connection periodically
                    try:
                        with self.serial_lock:
                            # Send a simple ping command
                            if self.ser.is_open:
                                self.ser.write(b"O\n")
                                time.sleep(0.1)
                    except Exception as e:
                        logger.warning(f"Connection check failed: {e}")
                        self.connected = False
            
            # Sleep until next check
            time.sleep(WATCHDOG_INTERVAL)
    
    def _reader_loop(self):
        """Background thread for reading from serial port with robust error handling."""
        if not self.connected or not self.ser:
            return
        
        consecutive_errors = 0
        
        while self.running:
            try:
                # Skip if not connected
                if not self.connected or not self.ser or not self.ser.is_open:
                    time.sleep(0.5)
                    continue
                
                with self.serial_lock:
                    if self.ser.in_waiting > 0:
                        # Read line from serial with timeout
                        response = self._read_with_timeout(0.5)
                        
                        if response:
                            timestamp = datetime.now()
                            logger.debug(f"Received: {response}")
                            
                            # Queue the response
                            self.response_queue.put((response, timestamp))
                            
                            # Process the response
                            self._process_response(response)
                            
                            # Reset consecutive error counter on success
                            consecutive_errors = 0
                
                # Brief sleep to prevent busy-waiting
                time.sleep(0.01)
                
            except Exception as e:
                consecutive_errors += 1
                logger.error(f"Error reading from serial ({consecutive_errors}): {e}")
                
                # If too many consecutive errors, try to reconnect
                if consecutive_errors > 5:
                    logger.warning("Too many consecutive read errors, reconnecting...")
                    self._attempt_reconnect()
                    consecutive_errors = 0
                
                time.sleep(0.1)
    
    def _read_with_timeout(self, timeout):
        """Read from serial with timeout to prevent blocking."""
        if not self.ser or not self.ser.is_open:
            return None
        
        start_time = time.time()
        buffer = bytearray()
        
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte:
                    buffer.extend(byte)
                    if byte == b'\n':
                        break
            else:
                time.sleep(0.01)
        
        return buffer.decode().strip() if buffer else None
    
    def _writer_loop(self):
        """Background thread for writing to serial port with robust error handling."""
        if not self.connected or not self.ser:
            return
        
        while self.running:
            try:
                # Skip if not connected
                if not self.connected or not self.ser or not self.ser.is_open:
                    time.sleep(0.5)
                    continue
                
                # Get command from queue with timeout
                try:
                    command, callback = self.send_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                
                # Send command to Arduino with retries
                success = False
                retries = 0
                response = None
                
                while not success and retries < self.max_retries:
                    try:
                        # Delay before sending to allow Arduino to be ready
                        time.sleep(self.pre_command_delay)
                        
                        with self.serial_lock:
                            # Send the command
                            self._send_raw(command)
                        
                        # Delay after sending to allow Arduino to process
                        time.sleep(self.post_command_delay)
                        
                        # Check for response, with longer timeout for move commands
                        # Move commands (F/B) need more time than turn commands (L/R)
                        if command.startswith('F') or command.startswith('B'):
                            timeout = 1.0  # Longer timeout for movement
                        else:
                            timeout = 0.5
                        
                        try:
                            response, timestamp = self.response_queue.get(timeout=timeout)
                            self.response_queue.task_done()
                            success = True
                        except queue.Empty:
                            # For some commands, no response is still a success
                            if command.startswith('S'):  # Stop command
                                success = True
                            else:
                                logger.warning(f"No response for command: {command} (retry {retries+1}/{self.max_retries})")
                        
                    except Exception as e:
                        logger.error(f"Error sending command: {command} - {e} (retry {retries+1}/{self.max_retries})")
                    
                    retries += 1
                    if not success and retries < self.max_retries:
                        time.sleep(self.retry_delay)
                
                # Call the callback with the response
                if callback:
                    if success and response:
                        callback(response)
                    else:
                        callback("No response or error")
                
                self.send_queue.task_done()
                
            except Exception as e:
                logger.error(f"Error in writer thread: {e}")
                time.sleep(0.1)
    
    def _attempt_reconnect(self):
        """Attempt to reconnect to the serial port after a failure."""
        if self.demo_mode:
            return
        
        logger.info("Attempting to reconnect to Arduino...")
        
        # Close the current connection if it's open
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")
        
        self.connected = False
        self.ser = None
        
        # Attempt to reconnect
        self._connect()
    
    def _send_raw(self, command, retries=0):
        """
        Send a raw command to the Arduino with optional retries.
        
        Args:
            command: Command string to send
            retries: Current retry count
        
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.connected or not self.ser or not self.ser.is_open:
            logger.debug(f"[NOT CONNECTED] Would send: {command}")
            return False
        
        try:
            # Ensure command ends with newline
            if not command.endswith('\n'):
                command += '\n'
            
            # Specially handle movement commands with higher priority
            if command.startswith(('F', 'B')):
                # Clear any pending data
                self.ser.reset_input_buffer()
                # Force write with flush
                self.ser.write(command.encode())
                self.ser.flush()
                # Log movement commands more prominently
                logger.info(f"Movement command sent: {command.strip()}")
            else:
                # Send normal command
                self.ser.write(command.encode())
                logger.debug(f"Sent: {command.strip()}")
            
            self.last_command = command.strip()
            self.last_command_time = datetime.now()
            return True
            
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            
            # Try to reconnect if sending fails
            if retries < 2:
                logger.warning(f"Attempting to reconnect and retry sending {command}")
                self._attempt_reconnect()
                if self.connected:
                    return self._send_raw(command, retries + 1)
            
            return False
    
    def _process_response(self, response):
        """
        Process a response from the Arduino.
        
        Args:
            response: Response string from Arduino
        """
        # Store the response
        self.last_response = response
        
        # Special debug for movement feedback
        if response.startswith(("Moving", "Turning")):
            logger.info(f"Movement feedback: {response}")
        
        # Check for special response formats
        if ":" in response:
            # Parse key:value format responses
            parts = response.split(":")
            if len(parts) >= 2:
                key = parts[0].strip()
                value = parts[1].strip()
                
                if key == "V":
                    # Battery voltage
                    try:
                        voltage = float(value)
                        self.event_bus.publish('battery_voltage', voltage)
                    except ValueError:
                        pass
                
                elif key == "US":
                    # Ultrasonic sensor readings
                    try:
                        readings = [float(v) for v in value.split(",")]
                        self.event_bus.publish('ultrasonic_readings', readings)
                    except ValueError:
                        pass
                
                elif key == "IR":
                    # IR sensor readings
                    try:
                        parts = value.split(",")
                        if len(parts) >= 2:
                            ir_back = int(parts[0])
                            ir_cliff = int(parts[1])
                            self.event_bus.publish('ir_readings', {
                                'back': ir_back,
                                'cliff': ir_cliff
                            })
                    except ValueError:
                        pass
        
        # Check for error messages
        elif "Error" in response or "error" in response:
            logger.warning(f"Arduino reported error: {response}")
            self.event_bus.publish('arduino_error', response)
        
        # Check for obstacle detection
        elif "Objected" in response or "Cliff Detected" in response:
            logger.warning(f"Obstacle detected: {response}")
            self.event_bus.publish('obstacle_detected', {
                'message': response,
                'timestamp': datetime.now().isoformat()
            })
        
        # Publish the raw response as an event
        self.event_bus.publish('arduino_response', response)
    
    # def send_command(self, command, callback=None):
    #     """
    #     Send a command to the Arduino.
        
    #     Args:
    #         command: Command string to send
    #         callback: Optional callback function to call with response
            
    #     Returns:
    #         Response message
    #     """
    #     if self.demo_mode:
    #         demo_response = self._generate_demo_response(command)
    #         if callback:
    #             callback(demo_response)
    #         return demo_response
        
    #     if not self.connected:
    #         if self._connect():  # Try to reconnect
    #             logger.info("Reconnected to Arduino")
    #         else:
    #             message = "Not connected to Arduino"
    #             if callback:
    #                 callback(message)
    #             return message
        
    #     # Log movement commands
    #     if command.startswith(('F', 'B')):
    #         direction = "forward" if command.startswith('F') else "backward"
    #         speed = command[1:] if len(command) > 1 else "default"
    #         logger.info(f"Queueing {direction} movement command with speed {speed}")
        
    #     # Queue the command for sending
    #     self.send_queue.put((command, callback))
        
    #     # For synchronous operation, could add a wait here
    #     return f"Command '{command}' queued for sending"
    

    """
Method update for SerialCommunicator to support high_priority parameter.
This is a targeted fix for the TypeError in beacon tracking system.
"""

    def send_command(self, command, callback=None, high_priority=False):
        """
        Send a command to the Arduino.
        
        Args:
            command: Command string to send
            callback: Optional callback function to call with response
            high_priority: Whether this is a high priority command
            
        Returns:
            Response message
        """
        if self.demo_mode:
            demo_response = self._generate_demo_response(command)
            if callback:
                callback(demo_response)
            return demo_response
        
        if not self.connected:
            if self._connect():  # Try to reconnect
                logger.info("Reconnected to Arduino")
            else:
                message = "Not connected to Arduino"
                if callback:
                    callback(message)
                return message
        
        # Log movement commands
        if command.startswith(('F', 'B')):
            direction = "forward" if command.startswith('F') else "backward"
            speed = command[1:] if len(command) > 1 else "default"
            logger.info(f"Queueing {direction} movement command with speed {speed}")
        elif command.startswith('S') and high_priority:
            logger.info("Queueing high-priority STOP command")
        
        # Queue the command for sending (priority handled during processing)
        self.send_queue.put((command, callback))
        
        # Handle stop commands immediately if high priority
        if high_priority and command.startswith('S'):
            try:
                with self.serial_lock:
                    if self.connected and self.ser and self.ser.is_open:
                        # Immediate stop - don't wait for queue processing
                        self._send_raw("S\n")
            except Exception as e:
                logger.error(f"Error sending immediate stop command: {e}")
        
        # For synchronous operation, could add a wait here
        return f"Command '{command}' queued for sending"
    
    def _generate_demo_response(self, command):
        """
        Generate a fake response for demo mode.
        
        Args:
            command: Command that would be sent
            
        Returns:
            Simulated response string
        """
        if not command:
            return "Error: Empty command"
        
        # Extract direction and speed
        direction = command[0].upper() if len(command) > 0 else '?'
        speed = int(command[1:]) if len(command) > 1 and command[1:].isdigit() else 0
        
        responses = {
            'F': f"Moving Forward at speed: {speed}",
            'B': f"Moving Backward at speed: {speed}",
            'L': f"Turning Left at speed: {speed}",
            'R': f"Turning Right at speed: {speed}",
            'S': "Stopped",
            'O': "Online"
        }
        
        response = responses.get(direction, f"Invalid command: {command}")
        
        # Simulate random sensor readings occasionally
        if direction in ['F', 'B', 'L', 'R'] and speed > 0:
            # 10% chance of sending a sensor reading
            if time.time() % 10 < 1:
                response += f"\nV:{12.1 + (time.time() % 1) * 0.5}"  # Voltage between 12.1 and 12.6
        
        logger.debug(f"[DEMO] Response: {response}")
        return response
    
    def request_sensor_data(self):
        """
        Request sensor data from the Arduino.
        
        Returns:
            True if request was sent, False otherwise
        """
        if not self.connected and not self.demo_mode:
            self._connect()  # Try to reconnect
        
        if self.connected or self.demo_mode:
            # Send a special command to request sensor data
            self.send_command("D")  # 'D' for data request
            return True
        return False
    
    def check_connection(self):
        """
        Check if the Arduino is still connected and responsive.
        
        Returns:
            True if connected, False otherwise
        """
        if self.demo_mode:
            return True
        
        if not self.connected or not self.ser or not self.ser.is_open:
            self._connect()  # Try to reconnect
        
        if not self.connected or not self.ser or not self.ser.is_open:
            return False
        
        # Send a ping command and wait for response
        response = []
        
        def collect_response(resp):
            response.append(resp)
        
        self.send_command("O", collect_response)  # 'O' for online check
        
        # Wait a bit for response
        time.sleep(0.5)
        
        return len(response) > 0 and "Online" in response[0]
    
    def is_connected(self):
        """
        Get the current connection status.
        
        Returns:
            Connection status string
        """
        if self.demo_mode:
            return "demo"
        
        if not self.connected or not self.ser or not self.ser.is_open:
            return "disconnected"
            
        return "connected"
    
    def close(self):
        """Close the serial connection and stop threads."""
        self.running = False
        
        # Stop the threads
        threads = [self.reader_thread, self.writer_thread, self.watchdog_thread]
        for thread in threads:
            if thread and thread.is_alive():
                thread.join(timeout=1.0)
        
        # Close the serial connection
        if self.connected and self.ser and self.ser.is_open:
            try:
                with self.serial_lock:
                    self.ser.close()
                logger.info("Serial connection closed")
            except Exception as e:
                logger.error(f"Error closing serial connection: {e}")
        
        self.connected = False
    
    """
    This is the fixed handle_command method for the SerialCommunicator class 
    to properly support high_priority parameter.
    """

    def handle_command(self, data):
        """
        Handle command event from event bus.
        
        Args:
            data: Command data dictionary
        """
        if not data or 'command' not in data:
            logger.warning("Received command event with no command")
            return
        
        command = data['command']
        callback = data.get('callback')
        high_priority = data.get('high_priority', False)  # Extract high_priority parameter
        
        self.send_command(command, callback, high_priority=high_priority)