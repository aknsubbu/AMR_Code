#!/usr/bin/env python3
"""
Standalone test script for BLE beacon scanning.
This script initializes the BeaconRegistry, EventBus, and BeaconScanner modules
and performs a scan for nearby BLE beacons.
"""
import os
import time
import logging
import tempfile
from pathlib import Path


# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("beacon_test")


# Simple EventBus implementation
class EventBus:
    """Simple event bus for inter-module communication."""
    
    def __init__(self):
        self.handlers = {}
        logger.info("EventBus initialized")
    
    def register(self, event_type, handler):
        """Register a handler for a specific event type."""
        if event_type not in self.handlers:
            self.handlers[event_type] = []
        self.handlers[event_type].append(handler)
        logger.debug(f"Registered handler for event: {event_type}")
    
    def publish(self, event_type, data=None):
        """Publish an event to all registered handlers."""
        logger.debug(f"Publishing event: {event_type}")
        if event_type in self.handlers:
            for handler in self.handlers[event_type]:
                try:
                    handler(data)
                except Exception as e:
                    logger.error(f"Error in event handler: {e}")


# Import the beacon module classes
from beacon_registry import BeaconRegistry
from beacon_scanner import BeaconScanner
from position_estimator import PositionEstimator


def scan_callback(data):
    """Callback for beacon scan completion."""
    logger.info(f"Scan complete! Found {len(data)} beacons:")
    for uuid, beacon_data in data.items():
        logger.info(f"  UUID: {uuid}, RSSI: {beacon_data['rssi']}, Address: {beacon_data['address']}")


def position_callback(data):
    """Callback for position updates."""
    position = data['position']
    logger.info(f"Position updated: x={position['x']}, y={position['y']}, orientation={position['orientation']:.1f}°")
    logger.info(f"Confidence: {position['confidence']:.2f}, Last update: {position['last_update']}")


def main():
    """Main test function."""
    # Create temporary database file
    temp_dir = tempfile.gettempdir()
    db_path = os.path.join(temp_dir, "beacon_test.db")
    config_dir = os.path.join(temp_dir, "beacon_config")
    
    # Create EventBus
    event_bus = EventBus()
    
    # Initialize BeaconRegistry
    logger.info(f"Initializing BeaconRegistry with DB at {db_path}")
    registry = BeaconRegistry(db_path=db_path, config_dir=config_dir)
    
    # Add some sample beacons (optional)

    # These would normally come from your configuration
    registry.add_beacon(uuid="8E348BFF-1D77-482B-9749-E63B0B45CF00", x=0, y=0, 
                       major=1, minor=10, description="Corner Beacon")
    registry.add_beacon(uuid="4f4da331-bb5d-4efe-a014-914f47130a72", x=1, y=1,
                       major=1, minor=10, description="Wall Beacon")

    
    # Print all registered beacons
    beacons = registry.get_all_beacons()
    logger.info(f"Registered {len(beacons)} beacons:")
    for beacon in beacons:
        logger.info(f"  {beacon['uuid']} at ({beacon['x']}, {beacon['y']}): {beacon['description']}")
    
    # Initialize BeaconScanner
    logger.info("Initializing BeaconScanner")
    scanner = BeaconScanner(event_bus=event_bus, scan_interval=5.0)
    
    # Initialize PositionEstimator
    logger.info("Initializing PositionEstimator")
    estimator = PositionEstimator(
        beacon_registry=registry,
        beacon_scanner=scanner,
        event_bus=event_bus,
        method='weighted'
    )
    
    # Register custom callbacks
    event_bus.register('beacon_scan_complete', scan_callback)
    event_bus.register('position_updated', position_callback)
    
    # Start scanning
    logger.info("Starting beacon scanner...")
    scanner.start_scanning()
    
    try:
        # Run for a minute, printing scan results
        logger.info("Scanning for beacons. Press Ctrl+C to stop...")
        for i in range(12):  # 12 * 5 seconds = 1 minute
            time.sleep(5)
            logger.info(f"Scan cycle {i+1}/12 complete")
            
            # Request position update
            event_bus.publish('request_position')
            
    except KeyboardInterrupt:
        logger.info("Scan interrupted by user")
    finally:
        # Stop scanning
        logger.info("Stopping beacon scanner...")
        scanner.stop_scanning()
        
        # Print final position
        position = estimator.get_current_position()
        logger.info(f"Final position: x={position['x']}, y={position['y']}")
        logger.info(f"Orientation: {position['orientation']:.1f}°, Confidence: {position['confidence']:.2f}")
        
        # Clean up (optional)
        logger.info("Test complete")


if __name__ == "__main__":
    main()