"""
Enhanced Serial communicator for interfacing with the Arduino.
Features binary protocol, high baud rate, priority commands, and improved reliability.
"""
import serial
import time
import logging
import threading
import queue
from datetime import datetime

logger = logging.getLogger(__name__)

# Protocol constants matching Arduino
START_MARKER = 0xFF
ACK_MARKER = 0xFE

class SerialCommunicator:
    """
    Manages binary serial communication with the Arduino.
    Handles command sending and response parsing with improved reliability.
    """
    
    def __init__(self, port, event_bus, baudrate=2000000, timeout=1.0, demo_mode=False):
        """
        Initialize the serial communicator.
        
        Args:
            port: Serial port name
            event_bus: EventBus for inter-module communication
            baudrate: Serial baud rate (defaults to 2000000 for high-speed communication)
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
        
        # Command processing delays - reduced for high-speed communication
        self.pre_command_delay = 0.05  # Delay before sending a command
        self.post_command_delay = 0.1  # Delay after sending a command
        
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
            return False
        
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
            
            # Open new connection with high baud rate
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            logger.info(f"Connected to Arduino on {self.port} at {self.baudrate} baud")
            
            # Important: Allow time for Arduino to reset and stabilize
            time.sleep(2)
            
            # Clear any pending data
            if self.ser.in_waiting:
                self.ser.reset_input_buffer()
            
            # Test the connection with a ping (Stop command)
            self._send_binary_command('S', 0, False)
            time.sleep(0.5)
            
            # Check for ACK response
            if self.ser.in_waiting:
                response = self._read_binary_response()
                if response:
                    logger.debug(f"Connection test response: {response}")
            
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
                            # Send a simple stop command to check connection
                            if self.ser.is_open:
                                self._send_binary_command('S', 0, False)
                                time.sleep(0.1)
                    except Exception as e:
                        logger.warning(f"Connection check failed: {e}")
                        self.connected = False
            
            # Sleep until next check
            time.sleep(WATCHDOG_INTERVAL)
    
    def _reader_loop(self):
        """Background thread for reading binary responses from serial port."""
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
                    if self.ser.in_waiting >= 3:  # We expect 3 bytes per response
                        # Read binary response
                        response = self._read_binary_response()
                        
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
    
    def _read_binary_response(self):
        """Read binary response from serial with timeout."""
        if not self.ser or not self.ser.is_open:
            return None
        
        try:
            # Check for ACK marker
            marker = self.ser.read(1)
            if not marker or marker[0] != ACK_MARKER:
                if marker and len(marker) > 0:
                    logger.warning(f"Unexpected marker received: {marker[0]:#x}, expected: {ACK_MARKER:#x}")
                return None
            
            # Read action and speed
            action_byte = self.ser.read(1)
            speed_byte = self.ser.read(1)
            
            if not action_byte or not speed_byte:
                logger.warning("Incomplete response received")
                return None
            
            action = chr(action_byte[0])
            speed = speed_byte[0]
            
            return {
                'action': action,
                'speed': speed
            }
        except Exception as e:
            logger.error(f"Error reading binary response: {e}")
            return None
    
    def _writer_loop(self):
        """Background thread for writing binary commands to serial port."""
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
                    command_data, callback = self.send_queue.get(timeout=0.5)
                except queue.Empty:
                    continue
                
                # Extract command components
                command = command_data.get('command', 'S')
                speed = command_data.get('speed', 0)
                high_priority = command_data.get('high_priority', False)
                
                # Send command to Arduino with retries
                success = False
                retries = 0
                response = None
                
                while not success and retries < self.max_retries:
                    try:
                        # Delay before sending to allow Arduino to be ready
                        time.sleep(self.pre_command_delay)
                        
                        with self.serial_lock:
                            # Send the binary command
                            self._send_binary_command(command, speed, high_priority)
                        
                        # Delay after sending to allow Arduino to process
                        time.sleep(self.post_command_delay)
                        
                        # Wait for ACK response
                        try:
                            response, timestamp = self.response_queue.get(timeout=0.5)
                            self.response_queue.task_done()
                            success = True
                        except queue.Empty:
                            # For some commands, no response is still a success
                            if command == 'S':  # Stop command
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
    
    def _send_binary_command(self, action, speed, high_priority):
        """
        Send a binary command to the Arduino.
        
        Args:
            action: Single character command (F, B, L, R, S)
            speed: Speed value (0-255)
            high_priority: Whether this is a high priority command
        
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.connected or not self.ser or not self.ser.is_open:
            logger.debug(f"[NOT CONNECTED] Would send: {action}, speed={speed}, priority={high_priority}")
            return False
        
        try:
            # Ensure valid speed range (0-255)
            speed = min(max(0, speed), 255)
            
            # Get action byte and set priority flag if needed
            action_byte = ord(action[0].upper())
            if high_priority:
                action_byte |= 0x80  # Set the high bit to flag as priority
            
            # Binary packet: START_MARKER, action_byte, speed
            command_bytes = bytes([START_MARKER, action_byte, speed])
            
            # Send command
            self.ser.write(command_bytes)
            
            # Log command details
            priority_str = "HIGH priority" if high_priority else "normal priority"
            logger.debug(f"Sent binary command: action={action}, speed={speed}, {priority_str}")
            
            self.last_command = f"{action},{speed},{high_priority}"
            self.last_command_time = datetime.now()
            return True
            
        except Exception as e:
            logger.error(f"Error sending binary command: {e}")
            return False
    
    def _process_response(self, response):
        """
        Process a binary response from the Arduino.
        
        Args:
            response: Response dictionary with action and speed
        """
        if not response or not isinstance(response, dict):
            return
        
        # Store the response
        self.last_response = response
        
        # Extract action and speed
        action = response.get('action', '')
        speed = response.get('speed', 0)
        
        # Log movement feedback
        if action in ['F', 'B', 'L', 'R']:
            direction_map = {
                'F': 'Forward',
                'B': 'Backward',
                'L': 'Left',
                'R': 'Right'
            }
            direction = direction_map.get(action, action)
            logger.info(f"Movement feedback: {direction} at speed {speed}")
        
        # Publish the response as an event
        self.event_bus.publish('arduino_response', {
            'action': action,
            'speed': speed,
            'timestamp': datetime.now().isoformat()
        })
    
    def send_command(self, command, speed=None, callback=None, high_priority=False):
        """
        Send a command to the Arduino.
        
        Args:
            command: Command character (F, B, L, R, S)
            speed: Speed value (0-255)
            callback: Optional callback function to call with response
            high_priority: Whether this is a high priority command
            
        Returns:
            Response message
        """
        # Extract speed if included in command (e.g., "F100")
        if speed is None and len(command) > 1 and command[1:].isdigit():
            action = command[0]
            speed = int(command[1:])
        else:
            action = command[0]
            speed = speed or 0  # Default to 0 if not specified
        
        if self.demo_mode:
            demo_response = self._generate_demo_response(action, speed, high_priority)
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
        if action in ['F', 'B']:
            direction = "forward" if action == 'F' else "backward"
            logger.info(f"Queueing {direction} movement command with speed {speed}, priority={'HIGH' if high_priority else 'normal'}")
        elif action == 'S' and high_priority:
            logger.info("Queueing high-priority STOP command")
        
        # Prepare command data dictionary
        command_data = {
            'command': action,
            'speed': speed,
            'high_priority': high_priority
        }
        
        # Queue the command for sending
        self.send_queue.put((command_data, callback))
        
        # Handle stop commands immediately if high priority
        if high_priority and action == 'S':
            try:
                with self.serial_lock:
                    if self.connected and self.ser and self.ser.is_open:
                        # Immediate stop - don't wait for queue processing
                        self._send_binary_command('S', 0, True)
            except Exception as e:
                logger.error(f"Error sending immediate stop command: {e}")
        
        return f"Command '{action}' with speed {speed} queued for sending"
    
    def _generate_demo_response(self, action, speed, high_priority):
        """
        Generate a fake response for demo mode.
        
        Args:
            action: Command action
            speed: Command speed
            high_priority: Whether this is a high priority command
            
        Returns:
            Simulated response dictionary
        """
        if not action:
            return {"error": "Empty command"}
        
        # Clean up action
        action = action.upper()[0]
        
        response = {
            'action': action,
            'speed': speed,
            'priority': high_priority
        }
        
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
            self.send_command('D', 0)  # 'D' for data request
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
        
        # Send a stop command and wait for response
        response = []
        
        def collect_response(resp):
            response.append(resp)
        
        self.send_command('S', 0, collect_response)
        
        # Wait a bit for response
        time.sleep(0.5)
        
        return len(response) > 0
    
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
    
    def handle_command(self, data):
        """
        Handle command event from event bus.
        
        Args:
            data: Command data dictionary
        """
        if not data or 'command' not in data:
            logger.warning("Received command event with no command")
            return
        
        command = data.get('command', '')
        speed = data.get('speed', 0)
        callback = data.get('callback')
        high_priority = data.get('high_priority', False)
        
        self.send_command(command, speed, callback, high_priority=high_priority)