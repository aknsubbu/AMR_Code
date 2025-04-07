"""
Command Translator for converting waypoints to Arduino motor commands.
With improved timing for reliable autonomous movement.
"""
import logging
import math
import time
import threading
from enum import Enum
from queue import Queue, Empty

logger = logging.getLogger(__name__)

class WaypointType(Enum):
    """Enum for different types of waypoints."""
    MOVE = "MOVE"
    TURN = "TURN"
    STOP = "STOP"
    WAIT = "WAIT"

class CommandTranslator:
    """
    Translates high-level waypoints into Arduino motor commands.
    Handles the execution of waypoint sequences.
    """
    
    def __init__(self, event_bus, default_speed=128):
        """
        Initialize the command translator.
        
        Args:
            event_bus: EventBus for inter-module communication
            default_speed: Default motor speed (0-255)
        """
        self.event_bus = event_bus
        self.default_speed = default_speed
        self.waypoint_queue = Queue()
        self.current_waypoint = None
        self.executing = False
        self.executor_thread = None
        
        # Configuration constants
        self.MIN_MOVE_TIME = 5.0  # Minimum time for any movement (seconds)
        self.MIN_TURN_TIME = 1.5  # Minimum time for any turn (seconds)
        self.COMMAND_DELAY = 0.5  # Delay between commands (seconds)
        
        # Calibration settings - adjust these based on your robot's actual performance
        self.MAX_SPEED_CM_PER_SEC = 20.0  # Maximum speed in cm/sec at full power
        self.MAX_TURN_SPEED_DEG_PER_SEC = 90.0  # Maximum turning speed in degrees/sec
        
        # Register event handlers
        self.event_bus.register('execute_waypoint', self.handle_execute_waypoint)
        self.event_bus.register('execute_waypoints', self.handle_execute_waypoints)
        self.event_bus.register('stop_execution', self.handle_stop_execution)
        self.event_bus.register('obstacle_detected', self.handle_obstacle_detected)
        
        logger.info("Command translator initialized")
    
    def execute_waypoint(self, waypoint):
        """
        Execute a single waypoint.
        
        Args:
            waypoint: Waypoint dictionary
            
        Returns:
            True if waypoint was executed, False otherwise
        """
        if not waypoint:
            logger.warning("Attempted to execute None waypoint")
            return False
        
        # Validate waypoint
        if not self._validate_waypoint(waypoint):
            logger.error(f"Invalid waypoint: {waypoint}")
            return False
        
        waypoint_type = waypoint.get('type')
        
        # Add a small delay between waypoints for stability
        time.sleep(self.COMMAND_DELAY)
        
        if waypoint_type == WaypointType.MOVE.value:
            return self._execute_move_waypoint(waypoint)
        
        elif waypoint_type == WaypointType.TURN.value:
            return self._execute_turn_waypoint(waypoint)
        
        elif waypoint_type == WaypointType.STOP.value:
            return self._execute_stop_waypoint(waypoint)
        
        elif waypoint_type == WaypointType.WAIT.value:
            return self._execute_wait_waypoint(waypoint)
        
        else:
            logger.error(f"Unknown waypoint type: {waypoint_type}")
            return False
    
    def execute_waypoints(self, waypoints):
        """
        Execute a sequence of waypoints.
        
        Args:
            waypoints: List of waypoint dictionaries
            
        Returns:
            True if execution started, False otherwise
        """
        if not waypoints:
            logger.warning("Attempted to execute empty waypoint list")
            return False
        
        # Stop any ongoing execution
        self.stop_execution()
        
        # Add waypoints to queue
        for waypoint in waypoints:
            if self._validate_waypoint(waypoint):
                self.waypoint_queue.put(waypoint)
            else:
                logger.warning(f"Skipping invalid waypoint: {waypoint}")
        
        # Start executor thread
        self.executing = True
        self.executor_thread = threading.Thread(target=self._executor_loop, daemon=True)
        self.executor_thread.start()
        
        logger.info(f"Started executing {len(waypoints)} waypoints")
        return True
    
    def stop_execution(self):
        """
        Stop the execution of waypoints.
        
        Returns:
            True if execution was stopped, False if not executing
        """
        if not self.executing:
            return False
        
        # Signal executor to stop
        self.executing = False
        
        # Wait for executor to finish
        if self.executor_thread and self.executor_thread.is_alive():
            self.executor_thread.join(timeout=2.0)
        
        # Clear the queue
        while not self.waypoint_queue.empty():
            try:
                self.waypoint_queue.get_nowait()
                self.waypoint_queue.task_done()
            except Empty:
                break
        
        # Send stop command to robot
        self._send_command('S0')
        
        logger.info("Waypoint execution stopped")
        return True
    
    def _executor_loop(self):
        """Background thread for executing waypoint sequences."""
        logger.debug("Waypoint executor started")
        
        while self.executing and not self.waypoint_queue.empty():
            try:
                # Get next waypoint
                waypoint = self.waypoint_queue.get_nowait()
                self.current_waypoint = waypoint
                
                # Log the waypoint being executed
                logger.debug(f"Executing waypoint: {waypoint}")
                
                # Execute waypoint
                success = self.execute_waypoint(waypoint)
                
                if not success:
                    logger.warning(f"Failed to execute waypoint: {waypoint}")
                
                # Mark waypoint as done
                self.waypoint_queue.task_done()
                
                # Publish progress
                remaining = self.waypoint_queue.qsize()
                self.event_bus.publish('waypoint_executed', {
                    'waypoint': waypoint,
                    'success': success,
                    'remaining': remaining
                })
                
                # Exit if execution stopped
                if not self.executing:
                    break
                
            except Empty:
                break
            except Exception as e:
                logger.error(f"Error in waypoint executor: {e}")
                break
        
        # If all waypoints executed or exception occurred
        self.executing = False
        self.current_waypoint = None
        
        # Publish execution completed event
        self.event_bus.publish('waypoints_completed', {
            'success': True,
            'remaining': self.waypoint_queue.qsize()
        })
        
        logger.debug("Waypoint executor finished")
    
    def _validate_waypoint(self, waypoint):
        """
        Validate a waypoint.
        
        Args:
            waypoint: Waypoint dictionary
            
        Returns:
            True if valid, False otherwise
        """
        if not isinstance(waypoint, dict):
            return False
        
        # Check required fields
        if 'type' not in waypoint:
            return False
        
        waypoint_type = waypoint['type']
        
        # Check type-specific requirements
        if waypoint_type == WaypointType.MOVE.value:
            return 'distance' in waypoint and 'direction' in waypoint
        
        elif waypoint_type == WaypointType.TURN.value:
            return 'angle' in waypoint
        
        elif waypoint_type == WaypointType.STOP.value:
            return True  # No additional requirements
        
        elif waypoint_type == WaypointType.WAIT.value:
            return 'duration' in waypoint
        
        return False
    
    def _execute_move_waypoint(self, waypoint):
        """
        Execute a move waypoint.
        
        Args:
            waypoint: Move waypoint dictionary
            
        Returns:
            True if executed successfully, False otherwise
        """
        direction = waypoint['direction']
        distance = waypoint['distance']
        speed = waypoint.get('speed', self.default_speed)
        
        # Calculate the time needed to travel the distance
        time_seconds = self._calculate_move_time(distance, speed)
        
        # Enforce minimum movement time for reliability
        time_seconds = max(time_seconds, self.MIN_MOVE_TIME)
        
        # Translate direction to Arduino command
        direction_cmd = self._get_direction_command(direction)
        
        # Log the command and duration
        logger.info(f"Moving {direction} at speed {speed} for {time_seconds:.2f} seconds (distance {distance:.1f}cm)")
        
        # Send the command
        self._send_command(f"{direction_cmd}{speed}")
        
        # Wait for the calculated time
        time.sleep(time_seconds)
        
        # Stop the robot
        self._send_command("S0")
        
        # Add a short delay after stopping to ensure stability
        time.sleep(0.5)
        
        return True
    
    def _execute_turn_waypoint(self, waypoint):
        """
        Execute a turn waypoint.
        
        Args:
            waypoint: Turn waypoint dictionary
            
        Returns:
            True if executed successfully, False otherwise
        """
        angle = waypoint['angle']
        speed = waypoint.get('speed', self.default_speed)
        
        # Determine turn direction
        turn_cmd = "L" if angle < 0 else "R"
        turn_direction = "left" if angle < 0 else "right"
        
        # Calculate turn time
        time_seconds = self._calculate_turn_time(abs(angle), speed)
        
        # Enforce minimum turn time for reliability
        time_seconds = max(time_seconds, self.MIN_TURN_TIME)
        
        # Log the command and duration
        logger.info(f"Turning {turn_direction} at speed {speed} for {time_seconds:.2f} seconds (angle {abs(angle):.1f}°)")
        
        # Send the command
        self._send_command(f"{turn_cmd}{speed}")
        
        # Wait for the calculated time
        time.sleep(time_seconds)
        
        # Stop the robot
        self._send_command("S0")
        
        # Add a short delay after stopping to ensure stability
        time.sleep(0.5)
        
        return True
    
    def _execute_stop_waypoint(self, waypoint):
        """
        Execute a stop waypoint.
        
        Args:
            waypoint: Stop waypoint dictionary
            
        Returns:
            True if executed successfully, False otherwise
        """
        # Log the command
        logger.info("Executing STOP waypoint")
        
        # Send stop command
        self._send_command("S0")
        
        # Wait a moment for the robot to fully stop
        time.sleep(0.5)
        
        return True
    
    def _execute_wait_waypoint(self, waypoint):
        """
        Execute a wait waypoint.
        
        Args:
            waypoint: Wait waypoint dictionary
            
        Returns:
            True if executed successfully, False otherwise
        """
        duration = waypoint['duration']
        
        # Log the command
        logger.info(f"Waiting for {duration:.2f} seconds")
        
        # Simply wait for the specified duration
        time.sleep(duration)
        
        return True
    
    def _calculate_move_time(self, distance, speed):
        """
        Calculate the time needed to move a certain distance.
        
        Args:
            distance: Distance in centimeters
            speed: Motor speed (0-255)
            
        Returns:
            Time in seconds
        """
        # Linear speed calculation based on power
        # Slower at lower power settings
        speed_factor = (speed / 255.0) ** 1.5  # Non-linear relationship between power and speed
        
        # Calculate time based on distance and speed
        estimated_speed = self.MAX_SPEED_CM_PER_SEC * speed_factor
        time_seconds = distance / max(1.0, estimated_speed)  # Avoid division by zero
        
        return max(0.5, time_seconds)  # At least 0.5 seconds
    
    def _calculate_turn_time(self, angle, speed):
        """
        Calculate the time needed to turn a certain angle.
        
        Args:
            angle: Angle in degrees (positive for right, negative for left)
            speed: Motor speed (0-255)
            
        Returns:
            Time in seconds
        """
        # Non-linear speed calculation based on power
        # Turning is often less efficient at lower power
        speed_factor = (speed / 255.0) ** 1.3  # Non-linear relationship between power and turning speed
        
        # Calculate time based on angle and speed
        estimated_turn_speed = self.MAX_TURN_SPEED_DEG_PER_SEC * speed_factor
        time_seconds = angle / max(1.0, estimated_turn_speed)  # Avoid division by zero
        
        return max(0.5, time_seconds)  # At least 0.5 seconds
    
    def _get_direction_command(self, direction):
        """
        Translate a direction string to Arduino command.
        
        Args:
            direction: Direction string ('forward', 'backward', 'left', 'right')
            
        Returns:
            Arduino command character
        """
        direction_map = {
            'forward': 'F',
            'backward': 'B',
            'left': 'L',
            'right': 'R'
        }
        
        return direction_map.get(direction.lower(), 'S')
    
    def _send_command(self, command):
        """
        Send a command to the Arduino via the event bus.
        
        Args:
            command: Command string
        """
        logger.debug(f"Sending command: {command}")
        self.event_bus.publish('serial_command', {
            'command': command
        })
    
    def handle_execute_waypoint(self, data):
        """
        Handle execute_waypoint event.
        
        Args:
            data: Waypoint data
        """
        if not data:
            logger.warning("Received execute_waypoint event with no data")
            return
        
        self.execute_waypoint(data)
    
    def handle_execute_waypoints(self, data):
        """
        Handle execute_waypoints event.
        
        Args:
            data: List of waypoints
        """
        if not data:
            logger.warning("Received execute_waypoints event with no data")
            return
        
        self.execute_waypoints(data)
    
    def handle_stop_execution(self, data=None):
        """
        Handle stop_execution event.
        
        Args:
            data: Optional event data
        """
        self.stop_execution()
    
    def handle_obstacle_detected(self, data):
        """
        Handle obstacle_detected event.
        
        Args:
            data: Obstacle data
        """
        # Stop execution if an obstacle is detected
        if self.executing:
            logger.warning(f"Stopping execution due to obstacle: {data.get('message', 'Unknown')}")
            self.stop_execution()
            
            # Publish obstacle avoidance needed event
            self.event_bus.publish('obstacle_avoidance_needed', {
                'current_waypoint': self.current_waypoint,
                'remaining_waypoints': list(self.waypoint_queue.queue),
                'obstacle': data
            })