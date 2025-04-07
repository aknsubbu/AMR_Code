#!/usr/bin/env python3
"""
Database initialization script.
Creates the SQLite database and populates it with initial data.
"""
import os
import sys
import sqlite3
import argparse
import json
from pathlib import Path
import logging

# Add parent directory to path to import modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def init_database(db_path, beacon_file=None, force=False):
    """
    Initialize the SQLite database with required tables.
    
    Args:
        db_path: Path to the SQLite database file
        beacon_file: Optional JSON file with beacon data
        force: Whether to overwrite an existing database
    
    Returns:
        True if successful, False otherwise
    """
    # Check if database already exists
    if os.path.exists(db_path) and not force:
        logger.error(f"Database already exists at {db_path}. Use --force to overwrite.")
        return False
    
    # Create parent directories if they don't exist
    Path(os.path.dirname(db_path)).mkdir(parents=True, exist_ok=True)
    
    try:
        # Create or connect to database
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        logger.info(f"Creating database at {db_path}")
        
        # Create beacons table
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS beacons (
            uuid TEXT PRIMARY KEY,
            major INTEGER,
            minor INTEGER,
            grid_x INTEGER,
            grid_y INTEGER,
            description TEXT,
            last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
        ''')
        
        # Create grid_map table
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS grid_map (
            x INTEGER,
            y INTEGER,
            navigable BOOLEAN,
            cost FLOAT,
            beacon_uuid TEXT,
            PRIMARY KEY (x, y),
            FOREIGN KEY (beacon_uuid) REFERENCES beacons (uuid)
        )
        ''')
        
        # Create settings table
        cursor.execute('''
        CREATE TABLE IF NOT EXISTS settings (
            key TEXT PRIMARY KEY,
            value TEXT,
            description TEXT,
            last_updated TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
        ''')
        
        conn.commit()
        
        # Import beacons if provided
        if beacon_file and os.path.exists(beacon_file):
            logger.info(f"Importing beacons from {beacon_file}")
            import_beacons(conn, beacon_file)
        
        # Add default settings
        cursor.execute('''
        INSERT OR REPLACE INTO settings (key, value, description) 
        VALUES 
        ('grid_size', '20', 'Default grid size'),
        ('default_speed', '128', 'Default motor speed')
        ''')
        
        conn.commit()
        conn.close()
        
        logger.info("Database initialization complete")
        return True
    
    except Exception as e:
        logger.error(f"Error initializing database: {e}")
        return False

def import_beacons(conn, beacon_file):
    """
    Import beacons from a JSON file.
    
    Args:
        conn: SQLite connection
        beacon_file: Path to JSON file with beacon data
    """
    try:
        with open(beacon_file, 'r') as f:
            beacons = json.load(f)
        
        cursor = conn.cursor()
        
        for beacon in beacons:
            uuid = beacon.get('uuid')
            if not uuid:
                logger.warning("Skipping beacon with no UUID")
                continue
            
            x = beacon.get('x', 0)
            y = beacon.get('y', 0)
            major = beacon.get('major', 0)
            minor = beacon.get('minor', 0)
            description = beacon.get('description', '')
            
            cursor.execute('''
            INSERT OR REPLACE INTO beacons 
            (uuid, major, minor, grid_x, grid_y, description)
            VALUES (?, ?, ?, ?, ?, ?)
            ''', (uuid, major, minor, x, y, description))
            
            # Add to grid_map too
            cursor.execute('''
            INSERT OR REPLACE INTO grid_map
            (x, y, navigable, cost, beacon_uuid)
            VALUES (?, ?, ?, ?, ?)
            ''', (x, y, 1, 1.0, uuid))
        
        conn.commit()
        logger.info(f"Imported {len(beacons)} beacons")
    
    except Exception as e:
        logger.error(f"Error importing beacons: {e}")

def create_sample_beacons(output_file):
    """
    Create a sample beacon JSON file.
    
    Args:
        output_file: Path to output JSON file
    
    Returns:
        True if successful, False otherwise
    """
    try:
        # Create sample beacons in a grid pattern
        beacons = []
        
        # Create a 4x4 grid of beacons
        for x in range(4):
            for y in range(4):
                uuid = f"beacon_{x}_{y}"
                description = f"Beacon at ({x},{y})"
                
                beacons.append({
                    'uuid': uuid,
                    'x': x,
                    'y': y,
                    'major': 1,
                    'minor': x*4 + y,
                    'description': description
                })
        
        # Write to file
        with open(output_file, 'w') as f:
            json.dump(beacons, f, indent=2)
        
        logger.info(f"Created sample beacon file with {len(beacons)} beacons at {output_file}")
        return True
    
    except Exception as e:
        logger.error(f"Error creating sample beacons: {e}")
        return False

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Initialize the robot database')
    parser.add_argument('--db', type=str, default='database/robot_db.sqlite',
                       help='Path to SQLite database')
    parser.add_argument('--beacons', type=str, 
                       help='Path to JSON file with beacon data')
    parser.add_argument('--force', action='store_true',
                       help='Force overwrite of existing database')
    parser.add_argument('--create-sample', type=str,
                       help='Create a sample beacon file at the specified path')
    
    args = parser.parse_args()
    
    if args.create_sample:
        create_sample_beacons(args.create_sample)
    else:
        init_database(args.db, args.beacons, args.force)