"""
Waypoint Generator for converting paths to executable waypoints.
"""
import logging
import math
from enum import Enum

logger = logging.getLogger(__name__)

class WaypointType(Enum):
    """Enum for different types of waypoints."""
    MOVE = "MOVE"
    TURN = "TURN"
    STOP = "STOP"
    WAIT = "WAIT"

class WaypointGenerator:
    """
    Generates executable waypoints from paths.
    Converts abstract paths into concrete movement commands.
    """
    
    def __init__(self, event_bus=None):
        """
        Initialize the waypoint generator.
        
        Args:
            event_bus: EventBus for inter-module communication
        """
        self.event_bus = event_bus
        self.grid_scale = 10  # cm per grid cell
        self.turn_threshold = 20  # degrees
        self.default_speed = 128  # Default motor speed (0-255)
        
        # Register event handlers if event bus is provided
        if self.event_bus:
            self.event_bus.register('path_found', self.handle_path_found)
        
        logger.info("Waypoint generator initialized")
    
    def generate_waypoints(self, path, current_orientation=0):
        """
        Generate waypoints from a path.
        
        Args:
            path: List of (x,y) tuples
            current_orientation: Current orientation in degrees (0-359)
            
        Returns:
            List of waypoint dictionaries
        """
        if not path or len(path) < 2:
            logger.warning(f"Path too short to generate waypoints: {len(path) if path else 0} points")
            return []
        
        waypoints = []
        current_pos = path[0]
        current_heading = current_orientation
        
        # Add initial waypoint to stop and ensure the robot is prepared
        waypoints.append({
            'type': WaypointType.STOP.value,
            'x': current_pos[0],
            'y': current_pos[1]
        })
        
        # Process each segment of the path
        for i in range(1, len(path)):
            next_pos = path[i]
            
            # Calculate direction to next waypoint
            dx = next_pos[0] - current_pos[0]
            dy = next_pos[1] - current_pos[1]
            distance = math.sqrt(dx*dx + dy*dy) * self.grid_scale
            
            # Calculate heading to next waypoint (0 = East, 90 = North)
            target_heading = (math.degrees(math.atan2(dy, dx)) + 360) % 360
            
            # Convert to robot heading (0 = North, clockwise)
            target_heading = (90 - target_heading) % 360
            
            # Calculate turn angle
            turn_angle = ((target_heading - current_heading + 180) % 360) - 180
            
            # Only add turn waypoint if the angle is significant
            if abs(turn_angle) > self.turn_threshold:
                waypoints.append({
                    'type': WaypointType.TURN.value,
                    'x': current_pos[0],
                    'y': current_pos[1],
                    'angle': turn_angle,
                    'target_heading': target_heading,
                    'speed': self._calculate_turn_speed(abs(turn_angle))
                })
                
                # Update current heading
                current_heading = target_heading
            
            # Add move waypoint
            move_direction = self._heading_to_direction(target_heading)
            waypoints.append({
                'type': WaypointType.MOVE.value,
                'x': next_pos[0],
                'y': next_pos[1],
                'distance': distance,
                'direction': move_direction,
                'speed': self._calculate_move_speed(distance)
            })
            
            # Update current position
            current_pos = next_pos
        
        # Add final stop waypoint
        waypoints.append({
            'type': WaypointType.STOP.value,
            'x': path[-1][0],
            'y': path[-1][1]
        })
        
        logger.info(f"Generated {len(waypoints)} waypoints from path with {len(path)} points")
        
        return waypoints
    
    def _calculate_move_speed(self, distance):
        """
        Calculate appropriate speed for a move waypoint.
        
        Args:
            distance: Distance to move in cm
            
        Returns:
            Speed value (0-255)
        """
        # Simple calculation: use default speed for most distances
        # Could be refined based on robot capabilities and battery state
        
        if distance < 10:
            # Very short distance, go slower
            return max(60, self.default_speed - 60)
        elif distance > 50:
            # Longer distance, go faster
            return min(255, self.default_speed + 40)
        else:
            # Normal distance
            return self.default_speed
    
    def _calculate_turn_speed(self, angle):
        """
        Calculate appropriate speed for a turn waypoint.
        
        Args:
            angle: Absolute angle to turn in degrees
            
        Returns:
            Speed value (0-255)
        """
        # Adjust turn speed based on angle
        if angle < 45:
            # Small turn, go slower for precision
            return max(60, self.default_speed - 60)
        elif angle > 120:
            # Large turn, go slower for stability
            return max(60, self.default_speed - 30)
        else:
            # Medium turn
            return max(60, self.default_speed - 20)
    
    def _heading_to_direction(self, heading):
        """
        Convert heading to direction string.
        
        Args:
            heading: Heading in degrees (0-359)
            
        Returns:
            Direction string ('forward', 'backward', 'left', 'right')
        """
        # Simplify heading to cardinal direction
        # 315-45 = forward
        # 45-135 = right
        # 135-225 = backward
        # 225-315 = left
        
        if heading >= 315 or heading < 45:
            return 'forward'
        elif heading >= 45 and heading < 135:
            return 'right'
        elif heading >= 135 and heading < 225:
            return 'backward'
        else:  # heading >= 225 and heading < 315
            return 'left'
    
    def handle_path_found(self, data):
        """
        Handle path_found event by generating waypoints.
        
        Args:
            data: Path data
        """
        if not data or 'path' not in data:
            logger.warning("Invalid path data")
            return
        
        path = data['path']
        
        # Get current orientation if available
        current_orientation = data.get('orientation', 0)
        
        # Generate waypoints
        waypoints = self.generate_waypoints(path, current_orientation)
        
        # Publish waypoints generated event
        if self.event_bus:
            self.event_bus.publish('waypoints_generated', {
                'waypoints': waypoints,
                'path': path,
                'count': len(waypoints)
            })