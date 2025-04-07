"""
Motion controller for translating high-level movement commands to low-level serial commands.
"""
import logging
import time
import threading
from enum import Enum

logger = logging.getLogger(__name__)

class MovementDirection(Enum):
    """Enum for movement directions."""
    FORWARD = 'F'
    BACKWARD = 'B'
    LEFT = 'L'
    RIGHT = 'R'
    STOP = 'S'

class MotionController:
    """
    Motion controller for robot movement.
    Translates high-level commands to serial commands.
    """
    
    def __init__(self, serial_communicator, event_bus, command_throttler=None):
        """
        Initialize the motion controller.
        
        Args:
            serial_communicator: SerialCommunicator instance
            event_bus: EventBus for inter-module communication
            command_throttler: CommandThrottler instance
        """
        self.serial = serial_communicator
        self.event_bus = event_bus
        self.command_throttler = command_throttler
        
        # Current state
        self.current_speed = 0
        self.current_direction = MovementDirection.STOP
        self.is_moving = False
        
        # Lock for thread safety
        self.state_lock = threading.Lock()
        
        # Register event handlers
        self.event_bus.register('motion_command', self.handle_motion_command)
        
        logger.info("Motion controller initialized")
    
    def send_command(self, command, high_priority=False):
        """
        Send a movement command.
        
        Args:
            command: Movement command string
            high_priority: Whether this is a high priority command
            
        Returns:
            Response message
        """
        # Check if command can be sent (if throttler is available)
        if self.command_throttler and not high_priority:
            if self.command_throttler.should_deduplicate(command):
                return "Command deduplicated"
            
            if not self.command_throttler.can_send_command(command):
                # Let the serial communicator handle queuing
                pass
            else:
                self.command_throttler.record_command(command)
        
        # Update current state
        self._update_state_from_command(command)
        
        # Send command to serial
        response = self.serial.send_command(command, high_priority=high_priority)
        
        # Publish state update event
        self._publish_state_update()
        
        return response
    
    def _update_state_from_command(self, command):
        """
        Update the internal state based on command.
        
        Args:
            command: Command string
        """
        if not command:
            return
            
        with self.state_lock:
            # Extract direction and speed
            direction = command[0].upper() if len(command) > 0 else 'S'
            speed = int(command[1:]) if len(command) > 1 and command[1:].isdigit() else 0
            
            # Update state
            try:
                self.current_direction = MovementDirection(direction)
            except ValueError:
                self.current_direction = MovementDirection.STOP
                
            self.current_speed = speed
            self.is_moving = self.current_direction != MovementDirection.STOP and self.current_speed > 0
    
    def _publish_state_update(self):
        """Publish state update event."""
        with self.state_lock:
            self.event_bus.publish('motion_state_update', {
                'direction': self.current_direction.name,
                'speed': self.current_speed,
                'is_moving': self.is_moving,
                'timestamp': time.time()
            })
    
    def handle_motion_command(self, data):
        """
        Handle motion command event.
        
        Args:
            data: Command data
        """
        command = data.get('command')
        high_priority = data.get('high_priority', False)
        
        if command:
            self.send_command(command, high_priority=high_priority)
    
    def move(self, direction, speed=128, high_priority=False):
        """
        Move the robot in a direction.
        
        Args:
            direction: MovementDirection or string
            speed: Movement speed (0-255)
            high_priority: Whether this is a high priority command
            
        Returns:
            Response message
        """
        # Normalize direction
        if isinstance(direction, str):
            try:
                direction = MovementDirection(direction.upper())
            except ValueError:
                logger.warning(f"Invalid direction: {direction}")
                return f"Invalid direction: {direction}"
        
        # Clamp speed
        speed = max(0, min(255, speed))
        
        # Create command
        command = f"{direction.value}{speed}"
        
        # Send command
        return self.send_command(command, high_priority=high_priority)
    
    def stop(self, high_priority=True):
        """
        Stop the robot.
        
        Args:
            high_priority: Whether this is a high priority command
            
        Returns:
            Response message
        """
        return self.move(MovementDirection.STOP, 0, high_priority=high_priority)
    
    def forward(self, speed=128):
        """Move forward."""
        return self.move(MovementDirection.FORWARD, speed)
    
    def backward(self, speed=128):
        """Move backward."""
        return self.move(MovementDirection.BACKWARD, speed)
    
    def turn_left(self, speed=128):
        """Turn left."""
        return self.move(MovementDirection.LEFT, speed)
    
    def turn_right(self, speed=128):
        """Turn right."""
        return self.move(MovementDirection.RIGHT, speed)
    
    def get_state(self):
        """
        Get the current motion state.
        
        Returns:
            Dict with motion state
        """
        with self.state_lock:
            return {
                'direction': self.current_direction.name,
                'speed': self.current_speed,
                'is_moving': self.is_moving,
                'timestamp': time.time()
            }