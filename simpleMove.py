"""
Integrated BeaconTrackerController with SerialCommunicator and MotionController.
Provides a complete implementation for tracking and approaching BLE beacons.
"""
import asyncio
import logging
import sys
import time
import threading
import queue
import signal
from datetime import datetime
from enum import Enum

# Import your existing modules
from modules.motion_controller.serial_communicator import SerialCommunicator
from modules.motion_controller.motion_controller import MotionController, MovementDirection

# Import the enhanced beacon tracker
from beaconTrackerController import BeaconTrackerController, TrackingState, CalibrationPhase

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler('beacon_tracker.log')
    ]
)
logger = logging.getLogger("BeaconTracker")

class CommandThrottler:
    """
    Command throttler to prevent command flooding.
    """
    def __init__(self, min_interval=0.1, deduplication_window=0.05):
        self.last_commands = {}
        self.min_interval = min_interval
        self.deduplication_window = deduplication_window
        self.lock = threading.Lock()
    
    def should_deduplicate(self, command):
        """Check if command should be deduplicated."""
        with self.lock:
            if command not in self.last_commands:
                return False
            
            last_time = self.last_commands.get(command, 0)
            current_time = time.time()
            
            # Deduplicate if very recent identical command
            return (current_time - last_time) < self.deduplication_window
    
    def can_send_command(self, command):
        """Check if command can be sent based on throttling rules."""
        with self.lock:
            if command not in self.last_commands:
                return True
            
            last_time = self.last_commands.get(command, 0)
            current_time = time.time()
            
            # Allow if enough time has passed
            return (current_time - last_time) >= self.min_interval
    
    def record_command(self, command):
        """Record that a command was sent."""
        with self.lock:
            self.last_commands[command] = time.time()

class EventBus:
    """Simple implementation of event bus."""
    def __init__(self):
        self.handlers = {}
        self.lock = threading.Lock()
        
    def register(self, event_type, handler):
        with self.lock:
            if event_type not in self.handlers:
                self.handlers[event_type] = []
            self.handlers[event_type].append(handler)
        
    def publish(self, event_type, data):
        handlers = []
        with self.lock:
            if event_type in self.handlers:
                handlers = self.handlers[event_type].copy()
        
        for handler in handlers:
            try:
                handler(data)
            except Exception as e:
                logger.error(f"Error in event handler for {event_type}: {e}")

class BeaconTrackerApp:
    """
    Main application for beacon tracking with integrated hardware control.
    """
    def __init__(self, serial_port='/dev/ttyUSB0', baud_rate=2000000, demo_mode=False):
        """
        Initialize the beacon tracker application.
        
        Args:
            serial_port: Serial port for Arduino communication
            baud_rate: Baud rate for serial communication
            demo_mode: Whether to run in demo mode without real hardware
        """
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.demo_mode = demo_mode
        
        # Components
        self.event_bus = EventBus()
        self.throttler = CommandThrottler(min_interval=0.1)
        self.serial = None
        self.motion = None
        self.tracker = None
        
        # Control flags
        self.running = False
        self.shutdown_event = asyncio.Event()
        
        # Register signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        logger.info("BeaconTrackerApp initialized")
    
    def _signal_handler(self, sig, frame):
        """Handle termination signals for graceful shutdown."""
        logger.info(f"Received signal {sig}, initiating shutdown...")
        self.running = False
        
        # Set shutdown event if we're in an asyncio context
        try:
            if not self.shutdown_event.is_set():
                loop = asyncio.get_event_loop()
                loop.call_soon_threadsafe(self.shutdown_event.set)
        except Exception:
            pass
    
    async def setup(self):
        """Set up all components."""
        logger.info("Setting up components...")
        
        # Create and connect serial communicator
        self.serial = SerialCommunicator(
            port=self.serial_port,
            event_bus=self.event_bus,
            baudrate=self.baud_rate,
            demo_mode=self.demo_mode
        )
        
        # Allow time for serial connection to stabilize
        await asyncio.sleep(2.0)
        
        # Create motion controller
        self.motion = MotionController(
            serial_communicator=self.serial,
            event_bus=self.event_bus,
            command_throttler=self.throttler
        )
        
        # Create beacon tracker with configuration
        tracker_config = {
            # Scanning parameters
            'scan_interval': 1.0,          # Interval between scans (seconds)
            'scan_duration': 0.5,          # Duration of each scan (seconds)
            'rssi_samples': 5,             # Number of samples for filtered RSSI
            'rssi_min_threshold': -80,     # Minimum RSSI threshold for detection
            
            # Movement parameters
            'rotation_speed': 100,         # Speed for rotation movements
            'movement_speed': 120,         # Speed for forward/backward movements
            'approach_speed': 100,         # Speed for approaching beacon
            
            # Calibration parameters
            'rotation_min_steps': 8,       # Minimum rotation calibration steps
            'rotation_max_steps': 12,      # Maximum rotation calibration steps
            'distance_step_time': 0.8,     # Time to move for each distance step
            'rssi_change_threshold': 3.0,  # Significant RSSI change threshold
            
            # Search parameters
            'search_rotation_steps': 6,    # Finer rotation steps for search (6 x 60°)
        }
        
        self.tracker = BeaconTrackerController(
            motion_controller=self.motion,
            event_bus=self.event_bus,
            config=tracker_config,
            logger=logger
        )
        
        # Register event handlers for monitoring
        self._register_event_handlers()
        
        # Start the tracker
        await self.tracker.start()
        
        logger.info("Components setup complete")
        return True
    
    def _register_event_handlers(self):
        """Register event handlers for monitoring."""
        # Serial and motion events
        self.event_bus.register('arduino_response', self._on_arduino_response)
        self.event_bus.register('motion_state_update', self._on_motion_update)
        
        # Beacon tracker events
        self.event_bus.register('beacon_tracker_scan_complete', self._on_scan_complete)
        self.event_bus.register('beacon_tracker_beacon_detected', self._on_beacon_detected)
        self.event_bus.register('beacon_tracker_calibration_complete', self._on_calibration_complete)
        self.event_bus.register('beacon_tracker_calibration_error', self._on_calibration_error)
        self.event_bus.register('beacon_tracker_tracking_start', self._on_tracking_start)
        self.event_bus.register('beacon_tracker_beacon_reached', self._on_beacon_reached)
    
    def _on_arduino_response(self, data):
        """Handle Arduino response events."""
        action = data.get('action', '')
        speed = data.get('speed', 0)
        
        # Only log significant responses to avoid spamming the logs
        if action and action != 'S' and speed > 0:
            logger.debug(f"Arduino: {action} at speed {speed}")
    
    def _on_motion_update(self, data):
        """Handle motion state update events."""
        direction = data.get('direction', '')
        speed = data.get('speed', 0)
        is_moving = data.get('is_moving', False)
        
        # Only log significant updates to avoid spamming the logs
        if is_moving and direction != 'STOP':
            logger.debug(f"Motion: {direction} at speed {speed}")
    
    def _on_scan_complete(self, data):
        """Handle scan complete events."""
        count = data.get('count', 0)
        logger.info(f"Scan complete: {count} beacons found")
    
    def _on_beacon_detected(self, data):
        """Handle beacon detected events."""
        # Only log occasionally to avoid spamming logs
        if time.time() % 5 < 0.1:  # Log roughly every 5 seconds
            name = data.get('name', 'Unknown')
            rssi = data.get('rssi', 0)
            logger.debug(f"Beacon: {name} (RSSI: {rssi})")
    
    def _on_calibration_complete(self, data):
        """Handle calibration complete events."""
        best_direction = data.get('best_direction', 'Unknown')
        best_angle = data.get('best_angle', 0)
        logger.info(f"Calibration complete: Best direction={best_direction}, angle={best_angle:.1f}°")
    
    def _on_calibration_error(self, data):
        """Handle calibration error events."""
        error = data.get('error', 'Unknown error')
        logger.error(f"Calibration error: {error}")
    
    def _on_tracking_start(self, data):
        """Handle tracking start events."""
        target = data.get('target', 'Unknown')
        logger.info(f"Tracking started for: {target}")
    
    def _on_beacon_reached(self, data):
        """Handle beacon reached events."""
        rssi = data.get('rssi', 0)
        logger.info(f"Beacon reached with RSSI: {rssi}")
    
    async def teardown(self):
        """Clean up all components."""
        logger.info("Tearing down components...")
        
        # Stop the tracker
        if self.tracker:
            await self.tracker.stop_tracking()
            await self.tracker.stop()
        
        # Close serial connection
        if self.serial:
            self.serial.close()
        
        logger.info("Teardown complete")
    
    async def run_scan_mode(self):
        """Run in beacon scanning mode."""
        logger.info("Running in scan mode...")
        
        # Initial scan to find beacons
        beacons = await self.tracker.scan_for_beacons(duration=5.0)
        
        if not beacons:
            logger.warning("No beacons found during scan")
            return
        
        # Display detected beacons
        logger.info(f"Found {len(beacons)} beacons:")
        for name, data in beacons.items():
            logger.info(f"  - {name}: RSSI {data['rssi']} dBm")
    
    async def run_tracking_mode(self, target_beacon=None, duration=60):
        """
        Run in beacon tracking mode.
        
        Args:
            target_beacon: Target beacon name or pattern (if None, user will be prompted)
            duration: Maximum tracking duration in seconds
        """
        # Initial scan to find beacons
        beacons = await self.tracker.scan_for_beacons(duration=5.0)
        
        if not beacons:
            logger.warning("No beacons found during initial scan")
            return
        
        # Display detected beacons
        logger.info(f"Found {len(beacons)} beacons:")
        for i, (name, data) in enumerate(beacons.items(), 1):
            logger.info(f"  {i}. {name}: RSSI {data['rssi']} dBm")
        
        # Select target beacon
        selected_beacon = target_beacon
        
        if not selected_beacon:
            # If no beacon specified, use the strongest one
            strongest_beacon = max(beacons.items(), key=lambda x: x[1]['rssi'])
            selected_beacon = strongest_beacon[0]
            logger.info(f"Automatically selected strongest beacon: {selected_beacon}")
        elif selected_beacon not in beacons:
            # Try to find matching beacon by pattern
            matching_beacons = [name for name in beacons.keys() 
                               if selected_beacon.lower() in name.lower()]
            
            if matching_beacons:
                selected_beacon = matching_beacons[0]
                logger.info(f"Selected beacon by pattern match: {selected_beacon}")
            else:
                logger.warning(f"Specified beacon '{selected_beacon}' not found")
                # Fall back to strongest beacon
                strongest_beacon = max(beacons.items(), key=lambda x: x[1]['rssi'])
                selected_beacon = strongest_beacon[0]
                logger.info(f"Falling back to strongest beacon: {selected_beacon}")
        
        # Set target and calibrate
        logger.info(f"Setting target beacon to: {selected_beacon}")
        self.tracker.set_target_beacon(selected_beacon)
        
        logger.info("Starting calibration...")
        calibration = await self.tracker.calibrate(force=True)
        
        if not calibration:
            logger.error("Calibration failed, aborting tracking")
            return
        
        # Start approaching the beacon
        logger.info("Starting approach to beacon...")
        await self.tracker.approach_beacon()
        
        # Track for specified duration
        start_time = time.time()
        interval = 1.0  # Status update interval
        
        logger.info(f"Tracking for up to {duration} seconds...")
        while self.running and time.time() - start_time < duration:
            # Check for shutdown signal
            if self.shutdown_event.is_set():
                break
            
            # Get current state
            state = self.tracker.get_state()
            
            # Log current status periodically
            if 'avg_rssi' in state and state['avg_rssi'] is not None:
                logger.info(f"RSSI: {state['avg_rssi']:.1f} dBm, State: {state['state']}")
            
            # Check for completion
            if state.get('avg_rssi', -100) > -45:
                logger.info("Very close to beacon, tracking complete")
                break
            
            # Check for signal loss
            if state.get('tracking_errors', 0) > 5:
                logger.warning("Too many tracking errors, attempting recalibration")
                await self.tracker.calibrate(force=True)
            
            await asyncio.sleep(interval)
        
        # Stop tracking
        await self.tracker.stop_tracking()
        logger.info("Tracking mode completed")
    
    async def run(self, mode='scan', target_beacon=None, duration=60):
        """
        Run the beacon tracker application.
        
        Args:
            mode: Operating mode ('scan' or 'track')
            target_beacon: Target beacon name or pattern for tracking mode
            duration: Maximum tracking duration in seconds
        """
        self.running = True
        
        try:
            # Set up components
            success = await self.setup()
            if not success:
                logger.error("Setup failed")
                return
            
            # Run the selected mode
            if mode == 'scan':
                await self.run_scan_mode()
            elif mode == 'track':
                await self.run_tracking_mode(target_beacon, duration)
            else:
                logger.error(f"Unknown mode: {mode}")
            
        except Exception as e:
            logger.error(f"Error running beacon tracker: {e}", exc_info=True)
        finally:
            # Ensure proper cleanup
            self.running = False
            await self.teardown()

async def main():
    """Main entry point for the application."""
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description="Beacon Tracker Application")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port for Arduino")
    parser.add_argument("--baud", type=int, default=2000000, help="Baud rate for serial communication")
    parser.add_argument("--mode", choices=["scan", "track"], default="scan", help="Operation mode")
    parser.add_argument("--target", help="Target beacon name or pattern for tracking")
    parser.add_argument("--duration", type=int, default=60, help="Maximum tracking duration in seconds")
    parser.add_argument("--demo", action="store_true", help="Run in demo mode without real hardware")
    
    args = parser.parse_args()
    
    # Create and run the application
    app = BeaconTrackerApp(
        serial_port=args.port,
        baud_rate=args.baud,
        demo_mode=args.demo
    )
    
    await app.run(
        mode=args.mode,
        target_beacon=args.target,
        duration=args.duration
    )

if __name__ == "__main__":
    # Run the application
    asyncio.run(main())