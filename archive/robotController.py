import asyncio
import logging
import math
import time
from typing import Dict, List, Optional, Tuple, Any
from enum import Enum

class RobotController:
    """
    High-level robot controller with position tracking.
    Integrates with MotionController for actual movement.
    """
    
    def __init__(self, motion_controller, event_bus=None, logger=None, config=None):
        """
        Initialize the robot controller.
        
        Args:
            motion_controller: MotionController instance
            event_bus: Optional event bus for inter-module communication
            logger: Logger instance (optional)
            config: Configuration parameters (optional)
        """
        self.motion = motion_controller
        self.event_bus = event_bus
        self.logger = logger or logging.getLogger("RobotController")
        
        # Default configuration
        self.config = {
            'position_update_interval': 0.1,  # Position update interval (seconds)
            'rotation_speed': 100,            # Default rotation speed
            'movement_speed': 128,            # Default movement speed
            'rotation_precision': 5,          # Rotation precision (degrees)
            'position_precision': 0.5,        # Position precision (units)
            'rotation_degrees_per_second': 45,# Estimated rotation speed (degrees/second)
            'movement_units_per_second': 5,   # Estimated movement speed (units/second)
            'max_acceleration': 0.5,          # Maximum acceleration (units/s^2)
            'max_deceleration': 1.0,          # Maximum deceleration (units/s^2)
        }
        
        # Update with provided config
        if config:
            self.config.update(config)
        
        # Position tracking
        self.position = (0, 0)  # x, y coordinates
        self.rotation = 0       # degrees, 0 = forward (positive x-axis)
        
        # Motion state
        self.is_moving = False
        self.current_speed = 0
        self.target_position = None
        self.target_rotation = None
        
        # Lock for thread safety
        self._lock = asyncio.Lock()
        
        # Position update task
        self._position_update_task = None
        self._running = False
        
        self.logger.info("RobotController initialized")
        
        # Register for motion state updates if event bus is available
        if self.event_bus:
            self.event_bus.register('motion_state_update', self._handle_motion_update)
    
    async def start(self):
        """
        Start the controller.
        
        Returns:
            Success flag
        """
        async with self._lock:
            if self._running:
                return True
                
            self._running = True
            
        # Start position update task
        self._position_update_task = asyncio.create_task(self._update_position_task())
        
        self.logger.info("RobotController started")
        return True
    
    async def stop(self):
        """
        Stop the controller.
        
        Returns:
            Success flag
        """
        async with self._lock:
            if not self._running:
                return True
                
            self._running = False
            
        # Cancel position update task
        if self._position_update_task and not self._position_update_task.done():
            self._position_update_task.cancel()
            try:
                await self._position_update_task
            except asyncio.CancelledError:
                pass
                
        # Stop robot motion
        self.motion.stop(high_priority=True)
        
        self.logger.info("RobotController stopped")
        return True
    
    def _handle_motion_update(self, data):
        """
        Handle motion state update event.
        
        Args:
            data: Motion state data dictionary
        """
        self.is_moving = data.get('is_moving', False)
        self.current_speed = data.get('speed', 0)
    
    async def _update_position_task(self):
        """
        Task for updating position based on motion.
        """
        last_update = time.time()
        
        try:
            while self._running:
                current_time = time.time()
                elapsed = current_time - last_update
                last_update = current_time
                
                # Update position based on motion
                if self.is_moving and self.current_speed > 0:
                    await self._update_position(elapsed)
                    
                # Check if we've reached target position/rotation
                await self._check_targets()
                
                # Wait for next update
                await asyncio.sleep(self.config['position_update_interval'])
                
        except asyncio.CancelledError:
            self.logger.info("Position update task cancelled")
            raise
            
        except Exception as e:
            self.logger.error(f"Error in position update task: {e}")
    
    async def _update_position(self, elapsed_time):
        """
        Update position based on current motion.
        
        Args:
            elapsed_time: Elapsed time since last update (seconds)
        """
        async with self._lock:
            # Get current motion state
            motion_state = self.motion.get_state()
            direction = motion_state['direction']
            speed = motion_state['speed']
            
            # Calculate distance moved
            speed_factor = speed / 255.0  # Normalize speed to 0-1
            distance = self.config['movement_units_per_second'] * speed_factor * elapsed_time
            
            # Update position based on direction and rotation
            if direction == 'FORWARD':
                # Move in direction of current rotation
                dx = distance * math.cos(math.radians(self.rotation))
                dy = distance * math.sin(math.radians(self.rotation))
                self.position = (self.position[0] + dx, self.position[1] + dy)
                
            elif direction == 'BACKWARD':
                # Move opposite to current rotation
                dx = distance * math.cos(math.radians(self.rotation))
                dy = distance * math.sin(math.radians(self.rotation))
                self.position = (self.position[0] - dx, self.position[1] - dy)
                
            elif direction == 'LEFT':
                # Rotate counter-clockwise
                rotation_amount = self.config['rotation_degrees_per_second'] * speed_factor * elapsed_time
                self.rotation = (self.rotation - rotation_amount) % 360
                
            elif direction == 'RIGHT':
                # Rotate clockwise
                rotation_amount = self.config['rotation_degrees_per_second'] * speed_factor * elapsed_time
                self.rotation = (self.rotation + rotation_amount) % 360
    
    async def _check_targets(self):
        """
        Check if target position or rotation has been reached.
        """
        async with self._lock:
            # Check target rotation
            if self.target_rotation is not None:
                # Calculate angle difference
                diff = abs((self.rotation - self.target_rotation + 180) % 360 - 180)
                
                if diff <= self.config['rotation_precision']:
                    # Target rotation reached
                    self.target_rotation = None
                    self.motion.stop()
                    
                    # Publish event
                    if self.event_bus:
                        self.event_bus.publish('robot_target_reached', {
                            'type': 'rotation',
                            'value': self.rotation
                        })
            
            # Check target position
            if self.target_position is not None:
                # Calculate distance to target
                dx = self.target_position[0] - self.position[0]
                dy = self.target_position[1] - self.position[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                if distance <= self.config['position_precision']:
                    # Target position reached
                    self.target_position = None
                    self.motion.stop()
                    
                    # Publish event
                    if self.event_bus:
                        self.event_bus.publish('robot_target_reached', {
                            'type': 'position',
                            'value': self.position
                        })
    
    async def get_position(self):
        """
        Get the current position.
        
        Returns:
            Position as (x, y) tuple
        """
        async with self._lock:
            return self.position
    
    async def get_rotation(self):
        """
        Get the current rotation angle.
        
        Returns:
            Rotation angle in degrees (0-359)
        """
        async with self._lock:
            return self.rotation
    
    async def set_position(self, x, y):
        """
        Manually set the current position.
        
        Args:
            x: X coordinate
            y: Y coordinate
            
        Returns:
            Success flag
        """
        async with self._lock:
            self.position = (x, y)
            self.logger.info(f"Position manually set to ({x}, {y})")
            return True
    
    async def set_rotation(self, angle):
        """
        Manually set the current rotation angle.
        
        Args:
            angle: Rotation angle in degrees
            
        Returns:
            Success flag
        """
        async with self._lock:
            # Normalize angle to 0-359
            self.rotation = angle % 360
            self.logger.info(f"Rotation manually set to {self.rotation}")
            return True
    
    async def rotate_to(self, target_angle):
        """
        Rotate to the specified angle.
        
        Args:
            target_angle: Target angle in degrees
            
        Returns:
            Success flag
        """
        # Normalize angle to 0-359
        target_angle = target_angle % 360
        
        async with self._lock:
            # Calculate angle difference
            diff = (target_angle - self.rotation) % 360
            
            # Determine shortest rotation direction
            if diff <= 180:
                # Rotate clockwise
                direction = 'RIGHT'
                amount = diff
            else:
                # Rotate counter-clockwise
                direction = 'LEFT'
                amount = 360 - diff
                
            # Skip if very small rotation
            if amount < self.config['rotation_precision']:
                return True
                
            # Set target
            self.target_rotation = target_angle
            
        # Calculate rotation time
        speed = self.config['rotation_speed']
        speed_factor = speed / 255.0
        rotation_time = amount / (self.config['rotation_degrees_per_second'] * speed_factor)
        
        # Execute rotation
        self.logger.info(f"Rotating {direction} to {target_angle} degrees")
        self.motion.move(direction, speed)
        
        # Wait for rotation to complete
        await asyncio.sleep(rotation_time)
        
        # Check if we reached the target
        current_rotation = await self.get_rotation()
        diff = abs((current_rotation - target_angle + 180) % 360 - 180)
        
        if diff > self.config['rotation_precision']:
            self.logger.warning(f"Rotation may not have completed precisely: current={current_rotation}, target={target_angle}")
            
        # Stop motion
        self.motion.stop()
        
        return True
    
    async def move_forward(self, distance):
        """
        Move forward by the specified distance.
        
        Args:
            distance: Distance to move
            
        Returns:
            Success flag
        """
        async with self._lock:
            # Calculate target position
            dx = distance * math.cos(math.radians(self.rotation))
            dy = distance * math.sin(math.radians(self.rotation))
            
            target_x = self.position[0] + dx
            target_y = self.position[1] + dy
            
            self.target_position = (target_x, target_y)
            
        # Calculate movement time
        speed = self.config['movement_speed']
        speed_factor = speed / 255.0
        movement_time = distance / (self.config['movement_units_per_second'] * speed_factor)
        
        # Execute movement
        self.logger.info(f"Moving forward {distance} units")
        self.motion.move('FORWARD', speed)
        
        # Wait for movement to complete
        await asyncio.sleep(movement_time)
        
        # Check if we reached the target
        current_position = await self.get_position()
        dx = current_position[0] - self.target_position[0]
        dy = current_position[1] - self.target_position[1]
        actual_distance = math.sqrt(dx*dx + dy*dy)
        
        if actual_distance > self.config['position_precision']:
            self.logger.warning(f"Movement may not have completed precisely: current={current_position}, target={self.target_position}")
            
        # Stop motion
        self.motion.stop()
        
        return True
    
    async def move_backward(self, distance):
        """
        Move backward by the specified distance.
        
        Args:
            distance: Distance to move
            
        Returns:
            Success flag
        """
        async with self._lock:
            # Calculate target position
            dx = distance * math.cos(math.radians(self.rotation))
            dy = distance * math.sin(math.radians(self.rotation))
            
            target_x = self.position[0] - dx
            target_y = self.position[1] - dy
            
            self.target_position = (target_x, target_y)
            
        # Calculate movement time
        speed = self.config['movement_speed']
        speed_factor = speed / 255.0
        movement_time = distance / (self.config['movement_units_per_second'] * speed_factor)
        
        # Execute movement
        self.logger.info(f"Moving backward {distance} units")
        self.motion.move('BACKWARD', speed)
        
        # Wait for movement to complete
        await asyncio.sleep(movement_time)
        
        # Stop motion
        self.motion.stop()
        
        return True
    
    async def move_to(self, target_position):
        """
        Move to the specified position with rotation.
        
        Args:
            target_position: Target position as (x, y) tuple
            
        Returns:
            Success flag
        """
        try:
            # Get current position
            current_position = await self.get_position()
            
            # Calculate vector to target
            dx = target_position[0] - current_position[0]
            dy = target_position[1] - current_position[1]
            
            # Calculate distance and angle
            distance = math.sqrt(dx*dx + dy*dy)
            target_angle = math.degrees(math.atan2(dy, dx)) % 360
            
            # Skip if already at target
            if distance < self.config['position_precision']:
                self.logger.info(f"Already at target position: {target_position}")
                return True
                
            # First rotate to face target
            await self.rotate_to(target_angle)
            
            # Then move forward to target
            await self.move_forward(distance)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error moving to position {target_position}: {e}")
            # Emergency stop
            self.motion.stop(high_priority=True)
            return False