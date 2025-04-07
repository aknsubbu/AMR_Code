"""
Path Planner module for finding optimal paths through the grid map.
Updated to use beacon names instead of UUIDs with optimized performance.
"""
import logging
import heapq
import math
import time
from collections import defaultdict
import numpy as np

logger = logging.getLogger(__name__)

class PathPlanner:
    """
    Plans paths through the grid map using A* algorithm.
    Can handle obstacle avoidance and path optimization.
    """
    
    def __init__(self, grid_map, event_bus=None, allow_diagonal=True):
        """
        Initialize the path planner.
        
        Args:
            grid_map: GridMap instance
            event_bus: EventBus for inter-module communication
            allow_diagonal: Whether to allow diagonal movement
        """
        self.grid_map = grid_map
        self.event_bus = event_bus
        self.allow_diagonal = allow_diagonal
        self.last_path = None
        self.last_start = None
        self.last_goal = None
        
        # Cache for paths to reduce recalculation
        self.path_cache = {}
        self.cache_max_size = 100
        
        # Register event handlers if event bus is provided
        if self.event_bus:
            self.event_bus.register('obstacle_detected', self.handle_obstacle_detected)
            self.event_bus.register('map_updated', self.handle_map_updated)
            self.event_bus.register('find_path', self.handle_find_path)
        
        logger.info("Path planner initialized with diagonal movement: %s", allow_diagonal)
    
    def find_path(self, start, goal, max_iterations=10000):
        """
        Find a path from start to goal using A* algorithm.
        
        Args:
            start: (x,y) tuple or beacon name
            goal: (x,y) tuple or beacon name
            max_iterations: Maximum number of iterations
            
        Returns:
            List of (x,y) tuples representing the path, or None if no path found
        """
        # Resolve beacon names to coordinates if necessary
        start_pos = self._resolve_position(start)
        goal_pos = self._resolve_position(goal)
        
        if not start_pos or not goal_pos:
            logger.error(f"Invalid start or goal position: {start} -> {goal}")
            return None
        
        # Store for potential replanning
        self.last_start = start_pos
        self.last_goal = goal_pos
        
        # Check if start and goal are navigable
        if not self.grid_map.is_navigable(*start_pos):
            # Try to find a nearby navigable cell
            start_pos = self.grid_map.find_nearest_navigable(*start_pos)
            if not start_pos:
                logger.error(f"Start position is not navigable and no nearby navigable cell found")
                return None
            logger.warning(f"Start position adjusted to nearest navigable cell: {start_pos}")
        
        if not self.grid_map.is_navigable(*goal_pos):
            # Try to find a nearby navigable cell
            goal_pos = self.grid_map.find_nearest_navigable(*goal_pos)
            if not goal_pos:
                logger.error(f"Goal position is not navigable and no nearby navigable cell found")
                return None
            logger.warning(f"Goal position adjusted to nearest navigable cell: {goal_pos}")
        
        # Check if path is in cache
        cache_key = (start_pos, goal_pos)
        if cache_key in self.path_cache:
            cached_path, timestamp = self.path_cache[cache_key]
            # Only use cached path if less than 5 seconds old
            if time.time() - timestamp < 5.0:
                logger.info(f"Using cached path from {start_pos} to {goal_pos}")
                return cached_path.copy()
        
        # Log planning attempt
        logger.info(f"Planning path from {start_pos} to {goal_pos}")
        
        # Run A* algorithm
        start_time = time.time()
        path = self._a_star(start_pos, goal_pos, max_iterations)
        elapsed = time.time() - start_time
        
        if path:
            logger.info(f"Path found with {len(path)} steps in {elapsed:.3f} seconds")
            
            # Optionally post-process the path
            path = self._optimize_path(path)
            
            # Store the path for potential replanning
            self.last_path = path.copy()
            
            # Cache the path
            self.path_cache[cache_key] = (path.copy(), time.time())
            # Trim cache if needed
            if len(self.path_cache) > self.cache_max_size:
                # Remove oldest entry
                oldest_key = min(self.path_cache.items(), key=lambda x: x[1][1])[0]
                del self.path_cache[oldest_key]
            
            # Publish path found event if event bus is available
            if self.event_bus:
                self.event_bus.publish('path_found', {
                    'start': start_pos,
                    'goal': goal_pos,
                    'path': path,
                    'path_length': len(path),
                    'planning_time': elapsed
                })
            
            return path
        else:
            logger.warning(f"No path found from {start_pos} to {goal_pos} after {elapsed:.3f} seconds")
            
            # Publish path not found event if event bus is available
            if self.event_bus:
                self.event_bus.publish('path_not_found', {
                    'start': start_pos,
                    'goal': goal_pos,
                    'planning_time': elapsed
                })
            
            return None
    
    def _resolve_position(self, position):
        """
        Resolve a position from different formats.
        
        Args:
            position: Position as (x,y) tuple, beacon name, or {'x': x, 'y': y} dict
            
        Returns:
            (x,y) tuple or None if invalid
        """
        if isinstance(position, tuple) and len(position) == 2:
            return position
        
        elif isinstance(position, dict) and 'x' in position and 'y' in position:
            return (position['x'], position['y'])
        
        elif isinstance(position, str):
            # Assume it's a beacon name (was UUID before)
            beacon_pos = self.grid_map.get_beacon_position(position)
            if beacon_pos:
                return beacon_pos
            logger.error(f"Beacon not found: {position}")
            return None
        
        else:
            logger.error(f"Invalid position format: {position}")
            return None
    
    def _a_star(self, start, goal, max_iterations):
        """
        A* pathfinding algorithm.
        
        Args:
            start: (x,y) tuple
            goal: (x,y) tuple
            max_iterations: Maximum number of iterations
            
        Returns:
            List of (x,y) tuples representing the path, or None if no path found
        """
        # Early exit for trivial case
        if start == goal:
            return [start]
        
        # If start and goal are close, check direct path first
        if self._manhattan_distance(start, goal) <= 3 and self._has_line_of_sight(start, goal):
            return [start, goal]
        
        # Initialize data structures
        open_set = []  # Priority queue of nodes to explore
        open_set_set = set()  # Set tracking what's in open_set for faster lookup
        closed_set = set()  # Set of explored nodes
        came_from = {}  # Mapping of node -> previous node
        g_score = defaultdict(lambda: float('inf'))  # Cost from start to node
        f_score = defaultdict(lambda: float('inf'))  # Estimated cost from start to goal through node
        
        # Initialize start node
        g_score[start] = 0
        f_score[start] = self._heuristic(start, goal)
        heapq.heappush(open_set, (f_score[start], start))
        open_set_set.add(start)
        
        iterations = 0
        
        while open_set and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_score
            _, current = heapq.heappop(open_set)
            open_set_set.remove(current)
            
            # Check if goal reached
            if current == goal:
                # Reconstruct path
                path = self._reconstruct_path(came_from, current)
                logger.debug(f"Path found in {iterations} iterations")
                return path
            
            # Mark as explored
            closed_set.add(current)
            
            # Check neighbors
            for neighbor in self.grid_map.get_neighbors(*current, self.allow_diagonal):
                # Skip if already explored
                if neighbor in closed_set:
                    continue
                
                # Get movement cost
                move_cost = self.grid_map.get_movement_cost(*current, *neighbor)
                if move_cost is None:
                    continue  # Invalid move
                
                # Calculate tentative g_score
                tentative_g = g_score[current] + move_cost
                
                # Skip if this path is worse
                if tentative_g >= g_score[neighbor]:
                    continue
                
                # This path is better, record it
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + self._heuristic(neighbor, goal)
                
                # Add to open set if not already there
                if neighbor not in open_set_set:
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    open_set_set.add(neighbor)
        
        # No path found
        logger.warning(f"No path found after {iterations} iterations")
        return None
    
    def _heuristic(self, pos, goal):
        """
        Heuristic function for A* (Manhattan distance with a slight tie-breaker).
        
        Args:
            pos: (x,y) tuple
            goal: (x,y) tuple
            
        Returns:
            Estimated cost from pos to goal
        """
        # Manhattan distance
        dx = abs(pos[0] - goal[0])
        dy = abs(pos[1] - goal[1])
        
        # Add a small tie-breaker to avoid searching many equal paths
        tie_breaker = 0.001 * (dx + dy)
        
        return dx + dy + tie_breaker
    
    def _manhattan_distance(self, pos1, pos2):
        """
        Calculate Manhattan distance between two points.
        
        Args:
            pos1: (x,y) tuple
            pos2: (x,y) tuple
            
        Returns:
            Manhattan distance
        """
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def _reconstruct_path(self, came_from, current):
        """
        Reconstruct path from came_from mapping.
        
        Args:
            came_from: Dict mapping node -> previous node
            current: Current (goal) node
            
        Returns:
            List of nodes from start to goal
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
    
    def _optimize_path(self, path):
        """
        Optimize the path by removing unnecessary waypoints.
        Uses a simple path smoothing algorithm.
        
        Args:
            path: List of (x,y) tuples
            
        Returns:
            Optimized path
        """
        if not path or len(path) <= 2:
            return path
        
        optimized = [path[0]]
        i = 0
        
        while i < len(path) - 1:
            # Look for line-of-sight opportunities to skip waypoints
            for j in range(len(path) - 1, i, -1):
                if self._has_line_of_sight(path[i], path[j]):
                    # We can go directly from i to j
                    if j > i + 1 and j < len(path) - 1:
                        optimized.append(path[j])
                    i = j
                    break
            i += 1
        
        # Ensure goal is included
        if optimized[-1] != path[-1]:
            optimized.append(path[-1])
        
        logger.debug(f"Path optimized from {len(path)} to {len(optimized)} waypoints")
        return optimized
    
    def _has_line_of_sight(self, start, end):
        """
        Check if there's a clear line of sight between two points.
        Uses Bresenham's line algorithm.
        
        Args:
            start: (x,y) tuple
            end: (x,y) tuple
            
        Returns:
            True if there's a clear line of sight, False otherwise
        """
        x0, y0 = int(start[0]), int(start[1])
        x1, y1 = int(end[0]), int(end[1])
        
        # Calculate Euclidean distance for early return
        dist = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
        if dist <= 1.5:  # For very close points, assume line of sight
            return True
        
        # Bresenham's line algorithm
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        # Check cells along the line
        while x0 != x1 or y0 != y1:
            # Check if current point is navigable
            if not self.grid_map.is_navigable(x0, y0):
                return False
            
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        
        return True
    
    def replan_path(self):
        """
        Replan the last path to avoid new obstacles.
        
        Returns:
            New path or None if replanning failed
        """
        if not self.last_start or not self.last_goal:
            logger.warning("Cannot replan: No previous path")
            return None
        
        logger.info(f"Replanning path from {self.last_start} to {self.last_goal}")
        
        # Clear path cache for these points to force recalculation
        cache_key = (self.last_start, self.last_goal)
        if cache_key in self.path_cache:
            del self.path_cache[cache_key]
            
        return self.find_path(self.last_start, self.last_goal)
    
    def handle_obstacle_detected(self, data):
        """
        Handle obstacle detection event.
        
        Args:
            data: Obstacle data
        """
        if self.last_path:
            # Try to replan if we have a current path
            logger.info(f"Obstacle detected: {data.get('message', 'Unknown')}. Replanning...")
            new_path = self.replan_path()
            
            if new_path:
                # Publish path updated event
                if self.event_bus:
                    self.event_bus.publish('path_updated', {
                        'path': new_path,
                        'reason': 'obstacle'
                    })
    
    def handle_map_updated(self, data):
        """
        Handle map updated event.
        
        Args:
            data: Map update data
        """
        # Clear path cache on significant map updates
        self.path_cache.clear()
        
        # Replan if we have a current path
        if self.last_path:
            logger.info("Map updated. Replanning current path...")
            new_path = self.replan_path()
            
            if new_path:
                # Publish path updated event
                if self.event_bus:
                    self.event_bus.publish('path_updated', {
                        'path': new_path,
                        'reason': 'map_update'
                    })
    
    def handle_find_path(self, data):
        """
        Handle find_path event.
        
        Args:
            data: Path request data
        """
        if not data or 'start' not in data or 'goal' not in data:
            logger.warning("Invalid path request data")
            return
        
        path = self.find_path(data['start'], data['goal'])
        
        # Result is published via 'path_found' or 'path_not_found' events
    
    def clear_cache(self):
        """
        Clear the path cache.
        """
        self.path_cache.clear()
        logger.debug("Path cache cleared")