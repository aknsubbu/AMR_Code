"""
Obstacle Detector for combining sensor data and detecting obstacles.
"""
import logging
import time
import threading
import math
from enum import Enum

logger = logging.getLogger(__name__)

class ObstacleType(Enum):
    """Types of obstacles that can be detected."""
    FRONT = "FRONT"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    BACK = "BACK"
    CLIFF = "CLIFF"
    UNKNOWN = "UNKNOWN"

class ObstacleDetector:
    """
    Combines data from multiple sensors to detect obstacles.
    Provides functions for checking obstacle presence and planning avoidance.
    """
    
    def __init__(self, ultrasonic_manager, ir_sensor_manager, event_bus):
        """
        Initialize the obstacle detector.
        
        Args:
            ultrasonic_manager: UltrasonicManager instance
            ir_sensor_manager: IRSensorManager instance
            event_bus: EventBus for inter-module communication
        """
        self.ultrasonic_manager = ultrasonic_manager
        self.ir_sensor_manager = ir_sensor_manager
        self.event_bus = event_bus
        
        # Obstacle tracking
        self.detected_obstacles = {}  # (x,y) -> {type, confidence, timestamp}
        self.obstacle_memory_time = 30.0  # How long to remember obstacles (seconds)
        
        # Current position and grid map (to be set later)
        self.current_position = (0, 0)
        self.grid_map = None
        
        # Lock for thread safety
        self.lock = threading.RLock()
        
        # Periodic cleanup
        self.cleanup_thread = None
        self.running = False
        
        # Register event handlers
        self.event_bus.register('obstacle_detected', self.handle_obstacle_detected)
        self.event_bus.register('position_updated', self.handle_position_updated)
        self.event_bus.register('set_grid_map', self.handle_set_grid_map)
        
        logger.info("Obstacle detector initialized")
        
        # Start cleanup thread
        self.start_cleanup()
    
    def start_cleanup(self, interval=5.0):
        """
        Start periodic cleanup of old obstacles.
        
        Args:
            interval: Cleanup interval in seconds
        """
        if self.running:
            logger.warning("Cleanup already running")
            return
        
        self.running = True
        self.cleanup_thread = threading.Thread(target=self._cleanup_loop, args=(interval,), daemon=True)
        self.cleanup_thread.start()
        logger.debug(f"Started obstacle cleanup thread with interval {interval}s")
    
    def stop_cleanup(self):
        """Stop periodic obstacle cleanup."""
        self.running = False
        if self.cleanup_thread:
            self.cleanup_thread.join(timeout=2.0)
        logger.debug("Stopped obstacle cleanup thread")
    
    def _cleanup_loop(self, interval):
        """
        Background thread for periodic obstacle cleanup.
        
        Args:
            interval: Cleanup interval in seconds
        """
        while self.running:
            self.cleanup_old_obstacles()
            time.sleep(interval)
    
    def cleanup_old_obstacles(self):
        """Remove obstacles that are older than the memory time."""
        now = time.time()
        removed = 0
        
        with self.lock:
            # Find obstacles to remove
            to_remove = []
            for pos, data in self.detected_obstacles.items():
                age = now - data['timestamp']
                if age > self.obstacle_memory_time:
                    to_remove.append(pos)
            
            # Remove them
            for pos in to_remove:
                del self.detected_obstacles[pos]
                removed += 1
                
                # Update grid map if available
                if self.grid_map:
                    self.grid_map.set_obstacle(pos[0], pos[1], False, temporary=True)
        
        if removed > 0:
            logger.debug(f"Cleaned up {removed} old obstacles")
    
    def handle_obstacle_detected(self, data):
        """
        Handle obstacle detection event.
        
        Args:
            data: Obstacle data
        """
        if not data:
            return
        
        # Extract data
        sensor = data.get('sensor', 'unknown')
        confidence = data.get('confidence', 0.8)
        timestamp = data.get('timestamp', time.time())
        message = data.get('message', 'Unknown obstacle')
        
        # Determine obstacle type
        if sensor == 'front':
            obstacle_type = ObstacleType.FRONT
        elif sensor == 'left':
            obstacle_type = ObstacleType.LEFT
        elif sensor == 'right':
            obstacle_type = ObstacleType.RIGHT
        elif sensor == 'ir_back':
            obstacle_type = ObstacleType.BACK
        elif sensor == 'ir_cliff':
            obstacle_type = ObstacleType.CLIFF
        else:
            obstacle_type = ObstacleType.UNKNOWN
        
        # Calculate obstacle position based on robot position and sensor
        obstacle_pos = self._calculate_obstacle_position(sensor, data.get('distance'))
        
        if obstacle_pos:
            # Add to tracked obstacles
            self._add_obstacle(obstacle_pos, obstacle_type, confidence, timestamp)
            
            # Log the detection
            logger.info(f"Obstacle detected: {obstacle_type.value} at {obstacle_pos}, confidence: {confidence:.2f}")
            
            # Update grid map if available
            if self.grid_map:
                self.grid_map.set_obstacle(obstacle_pos[0], obstacle_pos[1], True, temporary=True)
                
                # For cliff or high-confidence obstacles, also mark adjacent cells
                if obstacle_type == ObstacleType.CLIFF or confidence > 0.7:
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if dx == 0 and dy == 0:
                                continue
                            self.grid_map.set_obstacle(
                                obstacle_pos[0] + dx, 
                                obstacle_pos[1] + dy, 
                                True, 
                                temporary=True
                            )
            
            # Publish detailed obstacle info
            self.event_bus.publish('obstacle_info', {
                'position': obstacle_pos,
                'type': obstacle_type.value,
                'confidence': confidence,
                'timestamp': timestamp,
                'message': message
            })
    
    def _calculate_obstacle_position(self, sensor, distance=None):
        """
        Calculate the position of an obstacle based on sensor and robot position.
        
        Args:
            sensor: Sensor that detected the obstacle
            distance: Distance to obstacle in cm (for ultrasonic sensors)
            
        Returns:
            (x,y) tuple for obstacle position or None if cannot be calculated
        """
        if not hasattr(self, 'current_position') or not self.current_position:
            return None
        
        x, y = self.current_position
        
        # Default to 1 grid cell away if no distance provided
        grid_distance = 1
        
        if distance:
            # Convert cm to grid cells (assuming 10cm per cell)
            grid_distance = max(1, int(distance / 10))
        
        # Adjust position based on sensor
        if sensor == 'front':
            return (x, y + grid_distance)  # North
        elif sensor == 'left':
            return (x - grid_distance, y)  # West
        elif sensor == 'right':
            return (x + grid_distance, y)  # East
        elif sensor == 'ir_back':
            return (x, y - grid_distance)  # South
        elif sensor == 'ir_cliff':
            return (x, y + grid_distance)  # Cliff detected in front
        else:
            # Unknown sensor, can't determine position
            return None
    
    def _add_obstacle(self, position, obstacle_type, confidence, timestamp):
        """
        Add or update an obstacle in the tracking list.
        
        Args:
            position: (x,y) tuple
            obstacle_type: ObstacleType
            confidence: Confidence value (0.0-1.0)
            timestamp: Detection timestamp
        """
        with self.lock:
            # Check if obstacle already exists
            if position in self.detected_obstacles:
                # Update with higher confidence or newer timestamp
                existing = self.detected_obstacles[position]
                if confidence > existing['confidence'] or timestamp > existing['timestamp']:
                    existing.update({
                        'type': obstacle_type.value,
                        'confidence': confidence,
                        'timestamp': timestamp,
                        'count': existing['count'] + 1
                    })
            else:
                # Add new obstacle
                self.detected_obstacles[position] = {
                    'type': obstacle_type.value,
                    'confidence': confidence,
                    'timestamp': timestamp,
                    'count': 1
                }
    
    def handle_position_updated(self, position_data):
        """
        Handle position update event.
        
        Args:
            position_data: Position data
        """
        if not position_data or 'x' not in position_data or 'y' not in position_data:
            return
        
        # Update current position
        self.current_position = (position_data['x'], position_data['y'])
    
    def handle_set_grid_map(self, data):
        """
        Handle grid map update event.
        
        Args:
            data: Grid map data
        """
        if not data or 'grid_map' not in data:
            return
        
        # Set the grid map reference
        self.grid_map = data['grid_map']
    
    def is_path_clear(self, path):
        """
        Check if a path is clear of obstacles.
        
        Args:
            path: List of (x,y) coordinates
            
        Returns:
            True if path is clear, False if obstacles present
        """
        if not path:
            return True
        
        with self.lock:
            # Check each point in the path
            for pos in path:
                if pos in self.detected_obstacles:
                    logger.info(f"Obstacle detected in path at {pos}")
                    return False
                
                # If grid map is available, also check it
                if self.grid_map and not self.grid_map.is_navigable(*pos):
                    logger.info(f"Non-navigable cell in path at {pos}")
                    return False
        
        return True
    
    def suggest_avoidance_path(self, start, goal, blocked_path=None):
        """
        Suggest an alternative path to avoid obstacles.
        
        Args:
            start: Start position (x,y)
            goal: Goal position (x,y)
            blocked_path: Original blocked path
            
        Returns:
            List of waypoints for avoidance, or None if no avoidance possible
        """
        if not self.grid_map:
            logger.warning("Cannot suggest avoidance path without grid map")
            return None
        
        logger.info(f"Finding avoidance path from {start} to {goal}")
        
        # Strategy 1: Expand obstacles to ensure clearance
        self.grid_map.expand_obstacles(expansion=1)
        
        # Strategy 2: Find nearest navigable points if start/goal are blocked
        if not self.grid_map.is_navigable(*start):
            alt_start = self.grid_map.find_nearest_navigable(*start)
            if not alt_start:
                logger.warning("No navigable point near start")
                return None
            start = alt_start
        
        if not self.grid_map.is_navigable(*goal):
            alt_goal = self.grid_map.find_nearest_navigable(*goal)
            if not alt_goal:
                logger.warning("No navigable point near goal")
                return None
            goal = alt_goal
        
        # At this point, we need a path planner, but it's a circular dependency
        # Instead, publish a request for a new path
        self.event_bus.publish('request_path', {
            'start': start,
            'goal': goal,
            'reason': 'obstacle_avoidance'
        })
        
        # Return None since the path planning will be handled asynchronously
        return None
    
    def get_all_obstacles(self, string_keys=True):
        """
        Get all currently tracked obstacles.
        
        Args:
            string_keys: If True, convert tuple keys to strings for JSON compatibility
        
        Returns:
            Dict of obstacle data
        """
        with self.lock:
            if string_keys:
                # Create a new dict with string keys for JSON compatibility
                return {f"{pos[0]},{pos[1]}": data.copy() for pos, data in self.detected_obstacles.items()}
            else:
                # Return with original tuple keys (for internal use)
                return {pos: data.copy() for pos, data in self.detected_obstacles.items()}
            
    def check_immediate_surroundings(self):
        """
        Check if there are any obstacles in the immediate surroundings.
        Combines data from all sensors.
        
        Returns:
            Dict with obstacle status for each direction
        """
        # Get ultrasonic distances
        distances = self.ultrasonic_manager.get_current_distances()
        
        # Get IR sensor status
        ir_status = self.ir_sensor_manager.get_sensor_status()
        
        # Check each direction
        obstacles = {
            'front': distances[0] > 0 and distances[0] < 15,
            'left': distances[1] > 0 and distances[1] < 10,
            'right': distances[2] > 0 and distances[2] < 10,
            'back': ir_status['back_obstacle'],
            'cliff': ir_status['cliff']
        }
        
        # Check if any direction has an obstacle
        obstacles['any'] = any(obstacles.values())
        
        return obstacles