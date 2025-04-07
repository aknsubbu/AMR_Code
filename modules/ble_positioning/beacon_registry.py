"""
Beacon Registry for managing BLE beacon information.
"""
import os
import json
import logging
import sqlite3
from pathlib import Path

logger = logging.getLogger(__name__)

class BeaconRegistry:
    """
    Manages the registry of known BLE beacons.
    Stores beacon information in both SQLite and JSON format.
    Now using device name and MAC address as identifiers instead of UUID.
    """
    
    def __init__(self, db_path, config_dir='config'):
        """
        Initialize the beacon registry.
        
        Args:
            db_path: Path to SQLite database
            config_dir: Directory containing configuration files
        """
        self.db_path = db_path
        self.config_dir = config_dir
        self.json_path = os.path.join(config_dir, 'beacons.json')
        
        # Create directories if they don't exist
        Path(os.path.dirname(db_path)).mkdir(parents=True, exist_ok=True)
        Path(config_dir).mkdir(parents=True, exist_ok=True)
        
        # Initialize database and JSON file
        self._init_database()
        self._sync_json_to_db()
        
        logger.info(f"Beacon registry initialized with {len(self.get_all_beacons())} beacons")
    
    def _init_database(self):
        """Initialize the SQLite database with required tables."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Create beacons table with name as primary key and mac_address field
            cursor.execute('''
            CREATE TABLE IF NOT EXISTS beacons (
                name TEXT PRIMARY KEY,
                mac_address TEXT UNIQUE,
                grid_x INTEGER,
                grid_y INTEGER,
                description TEXT,
                last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
            ''')
            
            # Create grid_map table for navigation
            cursor.execute('''
            CREATE TABLE IF NOT EXISTS grid_map (
                x INTEGER,
                y INTEGER,
                navigable BOOLEAN,
                cost FLOAT,
                beacon_name TEXT,
                PRIMARY KEY (x, y),
                FOREIGN KEY (beacon_name) REFERENCES beacons (name)
            )
            ''')
            
            # If migrating from an older version with UUIDs, check if we need a migration
            cursor.execute("PRAGMA table_info(beacons)")
            columns = [column[1] for column in cursor.fetchall()]
            
            if 'uuid' in columns and 'name' not in columns:
                logger.info("Detected legacy database schema, performing migration...")
                self._migrate_from_uuid()
            
            conn.commit()
            conn.close()
            logger.debug("Database initialized successfully")
        except Exception as e:
            logger.error(f"Error initializing database: {e}")
            raise
    
    def _migrate_from_uuid(self):
        """Migrate database from UUID-based to name-based."""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # Create temporary table with new schema
            cursor.execute('''
            CREATE TABLE beacons_new (
                name TEXT PRIMARY KEY,
                mac_address TEXT UNIQUE,
                grid_x INTEGER,
                grid_y INTEGER,
                description TEXT,
                last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
            ''')
            
            # Copy data, using description or uuid as name
            cursor.execute('''
            INSERT INTO beacons_new (name, mac_address, grid_x, grid_y, description, last_updated)
            SELECT 
                COALESCE(description, uuid) AS name, 
                uuid AS mac_address, 
                grid_x, 
                grid_y, 
                description, 
                last_updated
            FROM beacons
            ''')
            
            # Update grid_map foreign keys
            cursor.execute('''
            CREATE TABLE grid_map_new (
                x INTEGER,
                y INTEGER,
                navigable BOOLEAN,
                cost FLOAT,
                beacon_name TEXT,
                PRIMARY KEY (x, y),
                FOREIGN KEY (beacon_name) REFERENCES beacons_new (name)
            )
            ''')
            
            cursor.execute('''
            INSERT INTO grid_map_new (x, y, navigable, cost, beacon_name)
            SELECT g.x, g.y, g.navigable, g.cost,
                (SELECT COALESCE(b.description, b.uuid) FROM beacons b WHERE b.uuid = g.beacon_uuid)
            FROM grid_map g
            ''')
            
            # Replace old tables with new ones
            cursor.execute("DROP TABLE beacons")
            cursor.execute("DROP TABLE grid_map")
            cursor.execute("ALTER TABLE beacons_new RENAME TO beacons")
            cursor.execute("ALTER TABLE grid_map_new RENAME TO grid_map")
            
            conn.commit()
            conn.close()
            logger.info("Database migration completed successfully")
        except Exception as e:
            logger.error(f"Error during database migration: {e}")
            raise
    
    def _sync_json_to_db(self):
        """
        Synchronize beacon data from JSON to database.
        If JSON file exists, load beacons from it.
        If not, create an empty JSON file.
        """
        try:
            if os.path.exists(self.json_path):
                # Load beacons from JSON and update database
                with open(self.json_path, 'r') as f:
                    beacons = json.load(f)
                
                conn = sqlite3.connect(self.db_path)
                cursor = conn.cursor()
                
                for beacon in beacons:
                    cursor.execute('''
                    INSERT OR REPLACE INTO beacons 
                    (name, mac_address, grid_x, grid_y, description)
                    VALUES (?, ?, ?, ?, ?)
                    ''', (
                        beacon['name'],
                        beacon.get('mac_address', ''),
                        beacon.get('x', 0),
                        beacon.get('y', 0),
                        beacon.get('description', '')
                    ))
                
                conn.commit()
                conn.close()
                logger.info(f"Loaded {len(beacons)} beacons from JSON")
            else:
                # Create empty JSON file
                with open(self.json_path, 'w') as f:
                    json.dump([], f, indent=2)
                logger.info("Created empty beacons JSON file")
        except Exception as e:
            logger.error(f"Error syncing JSON to database: {e}")
    
    def _sync_db_to_json(self):
        """Synchronize beacon data from database to JSON."""
        try:
            beacons = self.get_all_beacons()
            
            with open(self.json_path, 'w') as f:
                json.dump(beacons, f, indent=2)
            
            logger.debug(f"Synced {len(beacons)} beacons to JSON")
        except Exception as e:
            logger.error(f"Error syncing database to JSON: {e}")
    
    def add_beacon(self, name, mac_address, x, y, description=''):
        """
        Add or update a beacon in the registry.
        
        Args:
            name: Beacon name (primary identifier)
            mac_address: Beacon MAC address
            x: Grid X coordinate
            y: Grid Y coordinate
            description: Human-readable description
            
        Returns:
            True if successful, False otherwise
        """
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute('''
            INSERT OR REPLACE INTO beacons 
            (name, mac_address, grid_x, grid_y, description, last_updated)
            VALUES (?, ?, ?, ?, ?, CURRENT_TIMESTAMP)
            ''', (name, mac_address, x, y, description))
            
            # Update grid_map to mark the beacon location as navigable
            cursor.execute('''
            INSERT OR REPLACE INTO grid_map
            (x, y, navigable, cost, beacon_name)
            VALUES (?, ?, ?, ?, ?)
            ''', (x, y, True, 1.0, name))
            
            conn.commit()
            conn.close()
            
            # Sync to JSON
            self._sync_db_to_json()
            
            logger.info(f"Added/updated beacon {name} ({mac_address}) at grid ({x}, {y})")
            return True
        except Exception as e:
            logger.error(f"Error adding beacon: {e}")
            return False
    
    def remove_beacon(self, name):
        """
        Remove a beacon from the registry.
        
        Args:
            name: Beacon name
            
        Returns:
            True if successful, False otherwise
        """
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # First update grid_map to remove references to this beacon
            cursor.execute('''
            UPDATE grid_map SET beacon_name = NULL
            WHERE beacon_name = ?
            ''', (name,))
            
            # Then delete the beacon
            cursor.execute('DELETE FROM beacons WHERE name = ?', (name,))
            
            conn.commit()
            conn.close()
            
            # Sync to JSON
            self._sync_db_to_json()
            
            logger.info(f"Removed beacon {name}")
            return True
        except Exception as e:
            logger.error(f"Error removing beacon: {e}")
            return False
    
    def get_beacon(self, name):
        """
        Get information for a specific beacon by name.
        
        Args:
            name: Beacon name
            
        Returns:
            Beacon information as a dictionary or None if not found
        """
        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            
            cursor.execute('''
            SELECT name, mac_address, grid_x, grid_y, description, last_updated
            FROM beacons WHERE name = ?
            ''', (name,))
            
            row = cursor.fetchone()
            conn.close()
            
            if row:
                return {
                    'name': row['name'],
                    'mac_address': row['mac_address'],
                    'x': row['grid_x'],
                    'y': row['grid_y'],
                    'description': row['description'],
                    'last_updated': row['last_updated']
                }
            return None
        except Exception as e:
            logger.error(f"Error getting beacon {name}: {e}")
            return None
    
    def get_beacon_by_mac(self, mac_address):
        """
        Get information for a specific beacon by MAC address.
        
        Args:
            mac_address: Beacon MAC address
            
        Returns:
            Beacon information as a dictionary or None if not found
        """
        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            
            cursor.execute('''
            SELECT name, mac_address, grid_x, grid_y, description, last_updated
            FROM beacons WHERE mac_address = ?
            ''', (mac_address,))
            
            row = cursor.fetchone()
            conn.close()
            
            if row:
                return {
                    'name': row['name'],
                    'mac_address': row['mac_address'],
                    'x': row['grid_x'],
                    'y': row['grid_y'],
                    'description': row['description'],
                    'last_updated': row['last_updated']
                }
            return None
        except Exception as e:
            logger.error(f"Error getting beacon by MAC {mac_address}: {e}")
            return None
    
    def get_all_beacons(self):
        """
        Get all registered beacons.
        
        Returns:
            List of beacon dictionaries
        """
        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            
            cursor.execute('''
            SELECT name, mac_address, grid_x, grid_y, description, last_updated
            FROM beacons
            ORDER BY grid_x, grid_y
            ''')
            
            rows = cursor.fetchall()
            conn.close()
            
            beacons = []
            for row in rows:
                beacons.append({
                    'name': row['name'],
                    'mac_address': row['mac_address'],
                    'x': row['grid_x'],
                    'y': row['grid_y'],
                    'description': row['description'],
                    'last_updated': row['last_updated']
                })
            
            return beacons
        except Exception as e:
            logger.error(f"Error getting all beacons: {e}")
            return []
    
    def get_beacon_by_position(self, x, y):
        """
        Get beacon at a specific grid position.
        
        Args:
            x: Grid X coordinate
            y: Grid Y coordinate
            
        Returns:
            Beacon information as a dictionary or None if not found
        """
        try:
            conn = sqlite3.connect(self.db_path)
            conn.row_factory = sqlite3.Row
            cursor = conn.cursor()
            
            cursor.execute('''
            SELECT name, mac_address, grid_x, grid_y, description, last_updated
            FROM beacons WHERE grid_x = ? AND grid_y = ?
            ''', (x, y))
            
            row = cursor.fetchone()
            conn.close()
            
            if row:
                return {
                    'name': row['name'],
                    'mac_address': row['mac_address'],
                    'x': row['grid_x'],
                    'y': row['grid_y'],
                    'description': row['description'],
                    'last_updated': row['last_updated']
                }
            return None
        except Exception as e:
            logger.error(f"Error getting beacon at position ({x}, {y}): {e}")
            return None