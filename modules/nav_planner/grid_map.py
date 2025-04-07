"""
Grid Map module for representing the navigable space.
Updated to use beacon names instead of UUIDs.
"""
import logging
import sqlite3
import numpy as np
from collections import defaultdict

logger = logging.getLogger(__name__)

class GridCell:
    """Represents a single cell in the grid map."""
    
    def __init__(self, x, y, navigable=True, cost=1.0, beacon_name=None, obstacle=False):
        """
        Initialize a grid cell.
        
        Args:
            x: X coordinate
            y: Y coordinate
            navigable: Whether the cell is navigable
            cost: Movement cost through this cell (1.0 = normal)
            beacon_name: Name of a beacon at this location (if any)
            obstacle: Whether cell contains an obstacle
        """
        self.x = x
        self.y = y
        self.navigable = navigable
        self.cost = cost
        self.beacon_name = beacon_name
        self.obstacle = obstacle
        self.temp_obstacle = False  # For temporarily marking obstacles
    
    def __repr__(self):
        """String representation of the cell."""
        status = 'O' if self.obstacle else ('B' if self.beacon_name else ('.' if self.navigable else '#'))
        return f"{status}({self.x},{self.y})"

class GridMap:
    """
    Represents the navigation grid map.
    Stores information about beacons, obstacles, and navigable areas.
    """
    
    def __init__(self, beacon_registry, default_grid_size=20):
        """
        Initialize the grid map.
        
        Args:
            beacon_registry: BeaconRegistry instance
            default_grid_size: Default grid size if no beacons are defined
        """
        self.beacon_registry = beacon_registry
        self.grid = {}  # (x,y) -> GridCell
        self.obstacles = set()  # Set of (x,y) tuples
        self.beacons = {}  # (x,y) -> beacon_name
        self.default_grid_size = default_grid_size
        self.min_x = 0
        self.max_x = 0
        self.min_y = 0
        self.max_y = 0
        
        # Load the grid from the database
        self._load_grid()
        
        logger.info(f"Grid map initialized with {len(self.grid)} cells, {len(self.beacons)} beacons")
    
    def _load_grid(self):
        """Load the grid from the database."""
        try:
            # Get database path from beacon registry
            db_path = self.beacon_registry.db_path
            
            conn = sqlite3.connect(db_path)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            
            # Load beacons - updated for name-based identification
            cursor.execute('''
            SELECT name, grid_x, grid_y FROM beacons
            ''')
            
            rows = cursor.fetchall()
            
            # If no beacons, create a default grid
            if not rows:
                logger.warning("No beacons found in database, creating default grid")
                self._create_default_grid()
                return
            
            # Add beacons to the map
            for row in rows:
                x, y = row['grid_x'], row['grid_y']
                beacon_name = row['name']
                self.beacons[(x, y)] = beacon_name
                self.grid[(x, y)] = GridCell(x, y, navigable=True, beacon_name=beacon_name)
                
                # Update grid bounds
                self.min_x = min(self.min_x, x)
                self.max_x = max(self.max_x, x)
                self.min_y = min(self.min_y, y)
                self.max_y = max(self.max_y, y)
            
            # Load grid cells - updated for name-based identification
            cursor.execute('''
            SELECT x, y, navigable, cost, beacon_name FROM grid_map
            ''')
            
            rows = cursor.fetchall()
            
            for row in rows:
                x, y = row['x'], row['y']
                navigable = bool(row['navigable'])
                cost = row['cost']
                beacon_name = row['beacon_name']
                
                # Only add if not already added as a beacon
                if (x, y) not in self.grid:
                    self.grid[(x, y)] = GridCell(x, y, navigable, cost, beacon_name)
                    
                    # Update grid bounds
                    self.min_x = min(self.min_x, x)
                    self.max_x = max(self.max_x, x)
                    self.min_y = min(self.min_y, y)
                    self.max_y = max(self.max_y, y)
            
            # Fill in any gaps in the grid
            self._fill_grid_gaps()
            
            conn.close()
            
        except Exception as e:
            logger.error(f"Error loading grid: {e}")
            # Create a default grid if loading fails
            self._create_default_grid()
    
    def _create_default_grid(self):
        """Create a default grid if no beacons are defined."""
        size = self.default_grid_size
        
        # Create a simple square grid
        for x in range(size):
            for y in range(size):
                self.grid[(x, y)] = GridCell(x, y, navigable=True)
        
        self.min_x = 0
        self.max_x = size - 1
        self.min_y = 0
        self.max_y = size - 1
        
        logger.info(f"Created default grid of size {size}x{size}")
    
    def _fill_grid_gaps(self):
        """Fill in any gaps in the grid."""
        # Ensure all cells within bounds are defined
        for x in range(self.min_x, self.max_x + 1):
            for y in range(self.min_y, self.max_y + 1):
                if (x, y) not in self.grid:
                    # By default, cells are navigable
                    self.grid[(x, y)] = GridCell(x, y, navigable=True)
    
    def get_cell(self, x, y):
        """
        Get the cell at the specified coordinates.
        
        Args:
            x: X coordinate
            y: Y coordinate
            
        Returns:
            GridCell at (x,y) or None if outside grid
        """
        return self.grid.get((x, y))
    
    def set_obstacle(self, x, y, is_obstacle=True, temporary=True):
        """
        Set or clear an obstacle at the specified coordinates.
        
        Args:
            x: X coordinate
            y: Y coordinate
            is_obstacle: True to set as obstacle, False to clear
            temporary: If True, mark as temporary obstacle
            
        Returns:
            True if successful, False if coordinates are outside grid
        """
        cell = self.get_cell(x, y)
        if not cell:
            return False
        
        if temporary:
            cell.temp_obstacle = is_obstacle
        else:
            cell.obstacle = is_obstacle
            if is_obstacle:
                self.obstacles.add((x, y))
            else:
                self.obstacles.discard((x, y))
        
        return True
    
    def clear_temp_obstacles(self):
        """Clear all temporary obstacles."""
        for cell in self.grid.values():
            cell.temp_obstacle = False
    
    def is_navigable(self, x, y):
        """
        Check if a cell is navigable.
        
        Args:
            x: X coordinate
            y: Y coordinate
            
        Returns:
            True if navigable, False otherwise
        """
        cell = self.get_cell(x, y)
        if not cell:
            return False
        
        return cell.navigable and not cell.obstacle and not cell.temp_obstacle
    
    def get_movement_cost(self, x1, y1, x2, y2):
        """
        Get the cost of moving from (x1,y1) to (x2,y2).
        
        Args:
            x1, y1: Source coordinates
            x2, y2: Destination coordinates
            
        Returns:
            Movement cost or None if move is invalid
        """
        # Check if both cells exist and are navigable
        if not self.is_navigable(x1, y1) or not self.is_navigable(x2, y2):
            return None
        
        # Get the cells
        cell1 = self.get_cell(x1, y1)
        cell2 = self.get_cell(x2, y2)
        
        # Base cost is the average of the two cells
        base_cost = (cell1.cost + cell2.cost) / 2
        
        # Diagonal movement costs more
        if x1 != x2 and y1 != y2:
            base_cost *= 1.414  # sqrt(2)
        
        return base_cost
    
    def get_neighbors(self, x, y, allow_diagonal=True):
        """
        Get neighboring cells that are navigable.
        
        Args:
            x: X coordinate
            y: Y coordinate
            allow_diagonal: Whether to include diagonal neighbors
            
        Returns:
            List of (x,y) coordinates of navigable neighbors
        """
        neighbors = []
        
        # Define potential neighbors
        potential_neighbors = [
            (x+1, y), (x-1, y), (x, y+1), (x, y-1)  # Cardinal directions
        ]
        
        if allow_diagonal:
            potential_neighbors.extend([
                (x+1, y+1), (x+1, y-1), (x-1, y+1), (x-1, y-1)  # Diagonals
            ])
        
        # Check each potential neighbor
        for nx, ny in potential_neighbors:
            if self.is_navigable(nx, ny):
                neighbors.append((nx, ny))
        
        return neighbors
    
    def get_beacon_position(self, beacon_name):
        """
        Get the position of a beacon by name.
        
        Args:
            beacon_name: Beacon name
            
        Returns:
            (x,y) coordinates or None if not found
        """
        for pos, name in self.beacons.items():
            if name == beacon_name:
                return pos
        return None
    
    def get_beacon_at_position(self, x, y):
        """
        Get the beacon name at a position.
        
        Args:
            x: X coordinate
            y: Y coordinate
            
        Returns:
            Beacon name or None if no beacon at position
        """
        return self.beacons.get((x, y))
    
    def save_to_database(self):
        """Save the grid to the database."""
        try:
            # Get database path from beacon registry
            db_path = self.beacon_registry.db_path
            
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
            
            # Start a transaction
            cursor.execute('BEGIN TRANSACTION')
            
            # Clear existing grid cells
            cursor.execute('DELETE FROM grid_map')
            
            # Insert all grid cells
            for (x, y), cell in self.grid.items():
                cursor.execute('''
                INSERT INTO grid_map (x, y, navigable, cost, beacon_name)
                VALUES (?, ?, ?, ?, ?)
                ''', (
                    x, y, 
                    1 if cell.navigable else 0, 
                    cell.cost,
                    cell.beacon_name
                ))
            
            # Commit the transaction
            conn.commit()
            conn.close()
            
            logger.info(f"Saved grid with {len(self.grid)} cells to database")
            return True
        
        except Exception as e:
            logger.error(f"Error saving grid to database: {e}")
            return False
    
    def get_grid_dimensions(self):
        """
        Get the dimensions of the grid.
        
        Returns:
            Dict with min_x, max_x, min_y, max_y, width, height
        """
        return {
            'min_x': self.min_x,
            'max_x': self.max_x,
            'min_y': self.min_y,
            'max_y': self.max_y,
            'width': self.max_x - self.min_x + 1,
            'height': self.max_y - self.min_y + 1
        }
    
    def get_grid_as_array(self):
        """
        Convert the grid to a 2D numpy array for visualization.
        
        Returns:
            2D numpy array where:
            0 = not navigable
            1 = navigable
            2 = beacon
            3 = obstacle
        """
        width = self.max_x - self.min_x + 1
        height = self.max_y - self.min_y + 1
        
        grid_array = np.zeros((height, width), dtype=int)
        
        for (x, y), cell in self.grid.items():
            # Convert to array coordinates
            array_x = x - self.min_x
            array_y = y - self.min_y
            
            if 0 <= array_x < width and 0 <= array_y < height:
                if cell.obstacle or cell.temp_obstacle:
                    grid_array[array_y, array_x] = 3
                elif cell.beacon_name:
                    grid_array[array_y, array_x] = 2
                elif cell.navigable:
                    grid_array[array_y, array_x] = 1
                else:
                    grid_array[array_y, array_x] = 0
        
        return grid_array