"""
Enhanced Beacon Tracker Controller for locating and moving toward BLE beacons.
Integrates motion control and BLE scanning with advanced RSSI-based navigation.
"""
import asyncio
import logging
import time
import math
import hashlib
import statistics
import threading
from enum import Enum
from typing import Dict, List, Optional, Tuple, Callable, Any

from bleak import BleakScanner
from bleak.backends.scanner import AdvertisementData
from bleak.backends.device import BLEDevice

class TrackingState(Enum):
    """Enum for tracking states."""
    IDLE = 'idle'
    CALIBRATING = 'calibrating'
    SCANNING = 'scanning'
    TRACKING = 'tracking'
    APPROACHING = 'approaching'
    ERROR = 'error'

class CalibrationPhase(Enum):
    """Enum for calibration phases."""
    INIT = 'init'
    ROTATE_READINGS = 'rotate_readings'
    DISTANCE_READINGS = 'distance_readings'
    ANALYSIS = 'analysis'
    COMPLETE = 'complete'

class BeaconTrackerController:
    """
    Enhanced controller for tracking and approaching BLE beacons.
    Uses RSSI signal strength to navigate toward a specified beacon with
    improved calibration, position estimation, and movement optimization.
    """
    
    def __init__(
        self, 
        motion_controller, 
        event_bus=None,
        target_beacon_name=None,
        target_beacon_pattern=None,
        config=None,
        logger=None
    ):
        """
        Initialize the beacon tracker controller.
        
        Args:
            motion_controller: Motion controller for robot movement
            event_bus: Optional event bus for publishing status updates
            target_beacon_name: Name of the beacon to track (case insensitive, exact match)
            target_beacon_pattern: Pattern to match in beacon names (case insensitive, partial match)
            config: Configuration parameters (optional)
            logger: Logger instance (optional)
        """
        self.motion_controller = motion_controller
        self.event_bus = event_bus
        self.target_beacon_name = target_beacon_name
        self.target_beacon_pattern = target_beacon_pattern
        self.logger = logger or logging.getLogger("BeaconTracker")
        
        # Default configuration
        self.config = {
            # Scanning parameters
            'scan_interval': 1.0,          # Interval between scans (seconds)
            'scan_duration': 0.5,          # Duration of each scan (seconds)
            'rssi_samples': 7,             # Number of samples for filtered RSSI
            'rssi_min_threshold': -80,     # Minimum RSSI threshold for detection
            
            # RSSI measurement
            'rssi_interval': 0.05,         # Interval between RSSI samples
            'rssi_timeout': 2.0,           # Timeout for RSSI measurements
            'rssi_change_threshold': 3.0,  # Significant RSSI change threshold
            
            # Rotation calibration
            'rotation_min_steps': 8,        # Minimum rotation calibration steps
            'rotation_max_steps': 12,       # Maximum rotation calibration steps
            'rotation_speed': 100,          # Speed for rotation
            
            # Distance calibration
            'distance_max_steps': 5,        # Maximum distance calibration steps
            'distance_step_time': 1.0,      # Time to move for each distance step
            'approach_speed': 128,          # Speed for approaching beacon
            
            # Tracking
            'calibration_max_age': 300,     # Maximum calibration age (seconds)
            'tracking_interval': 1.0,       # Interval for continuous tracking
            'max_tracking_errors': 3,       # Maximum tracking errors before recalibration
            'movement_speed': 100,          # Default movement speed
            
            # Search parameters
            'search_rotation_steps': 4,     # Number of rotation steps for search (360°/steps)
            'search_max_spiral_steps': 5,   # Maximum steps in spiral search
            
            # BLE parameters
            'filter_duplicates': True,      # Filter duplicate advertisements
        }
        
        # Update with provided config
        if config:
            self.config.update(config)
        
        # BLE scanner
        self.scanner = None
        self.scanning_task = None
        self.scan_lock = threading.Lock()
        
        # Beacon tracking state
        self.tracking_state = TrackingState.IDLE
        self.calibration_phase = CalibrationPhase.INIT
        self.detected_beacons = {}          # Map of beacon names to their data
        self.target_beacon_data = None      # Current target beacon data
        
        # RSSI tracking
        self.current_rssi = None            # Most recent RSSI reading
        self.rssi_readings = []             # List of recent RSSI readings
        self.rssi_by_direction = {}         # RSSI readings by direction
        
        # Calibration data
        self.rotation_map = {}              # angle -> rssi
        self.distance_map = {}              # distance -> rssi
        self.calibration_data = {
            'rotation_map': {},             # Maps degrees to RSSI values
            'distance_map': {},             # Maps distances to RSSI values
            'best_direction': None,         # Best movement direction
            'best_angle': None,             # Best rotation angle
            'signal_peak': None,            # Maximum signal strength
            'timestamp': 0,                 # When calibration was performed
            'environment_hash': None,       # Hash of environment conditions
        }
        
        # Position and tracking
        self.current_position = (0, 0)      # x, y coordinates (relative)
        self.current_rotation = 0           # degrees, 0 = forward
        self.estimated_position = None      # (distance, angle) to beacon
        self.is_tracking = False            # Whether continuous tracking is active
        self.tracking_task = None           # Async task for continuous tracking
        self.tracking_errors = 0            # Counter for tracking errors
        
        # Control flags
        self.running = False
        self.paused = False
        
        # Initialize rssi_by_direction with movement directions
        try:
            for direction in self.motion_controller.current_direction.__class__:
                self.rssi_by_direction[direction.name] = []
        except (AttributeError, TypeError):
            # Default direction names if MotionController doesn't have direction enum
            for direction in ['FORWARD', 'BACKWARD', 'LEFT', 'RIGHT', 'STOP']:
                self.rssi_by_direction[direction] = []
        
        self.logger.info("Enhanced Beacon Tracker Controller initialized")
    
    async def start(self):
        """
        Start the beacon tracker.
        
        Returns:
            Success flag
        """
        if self.running:
            self.logger.warning("Beacon tracker already running")
            return False
        
        self.running = True
        self.tracking_state = TrackingState.IDLE
        
        # Initialize BLE scanner
        self.scanner = BleakScanner()
        self.scanner.register_detection_callback(self._on_device_detected)
        
        self._publish_event('tracker_start', {})
        
        self.logger.info("Beacon tracker started")
        return True
    
    async def stop(self):
        """
        Stop the beacon tracker and any ongoing operations.
        
        Returns:
            Success flag
        """
        if not self.running:
            return True
        
        self.running = False
        
        # Stop continuous tracking if active
        await self.stop_tracking()
        
        # Stop any ongoing scanning
        if self.scanner:
            await self.scanner.stop()
        
        # Stop the robot
        self.motion_controller.stop(high_priority=True)
        
        self.tracking_state = TrackingState.IDLE
        self._publish_event('tracker_stop', {})
        
        self.logger.info("Beacon tracker stopped")
        return True
    
    async def pause(self):
        """
        Pause the tracker temporarily.
        
        Returns:
            Success flag
        """
        if not self.running or self.paused:
            return False
        
        self.paused = True
        self.motion_controller.stop(high_priority=True)
        
        self._publish_event('tracker_pause', {})
        self.logger.info("Beacon tracker paused")
        return True
    
    async def resume(self):
        """
        Resume the tracker after pausing.
        
        Returns:
            Success flag
        """
        if not self.running or not self.paused:
            return False
        
        self.paused = False
        
        self._publish_event('tracker_resume', {})
        self.logger.info("Beacon tracker resumed")
        return True
    
    def _publish_event(self, event_type, data):
        """Publish event to event bus if available."""
        if self.event_bus:
            try:
                self.event_bus.publish(f'beacon_tracker_{event_type}', data)
            except Exception as e:
                self.logger.warning(f"Error publishing event: {e}")
    
    def set_target_beacon(self, beacon_name: str):
        """
        Set the target beacon to track by exact name match.
        
        Args:
            beacon_name: Name of the beacon to track (case insensitive)
            
        Returns:
            Success flag
        """
        if not beacon_name:
            self.logger.error("Invalid beacon name")
            return False
            
        self.target_beacon_name = beacon_name
        self.target_beacon_pattern = None  # Clear any pattern when setting exact name
        self.target_beacon_data = None  # Reset any existing data
        
        self._publish_event('beacon_set', {'beacon_name': beacon_name})
        self.logger.info(f"Target beacon set to: {beacon_name}")
        return True
    
    def set_target_beacon_pattern(self, pattern: str):
        """
        Set a pattern to match in beacon names.
        
        Args:
            pattern: String pattern to look for in beacon names (case insensitive)
            
        Returns:
            Success flag
        """
        if not pattern:
            self.logger.error("Invalid beacon pattern")
            return False
            
        self.target_beacon_pattern = pattern
        self.target_beacon_name = None  # Clear any exact name when setting pattern
        self.target_beacon_data = None  # Reset any existing data
        
        self._publish_event('beacon_set', {'beacon_pattern': pattern})
        self.logger.info(f"Target beacon pattern set to: {pattern}")
        return True
    
    def find_beacon_by_name(self, name: str = None) -> Optional[Dict]:
        """
        Find a beacon by name in the list of detected beacons.
        
        Args:
            name: Name to search for (default: use target_beacon_name)
            
        Returns:
            Beacon data dictionary or None if not found
        """
        if not name and not self.target_beacon_name and not self.target_beacon_pattern:
            return None
            
        search_name = name or self.target_beacon_name
        
        # First try exact match if we have a name
        if search_name:
            for beacon_name, beacon_data in self.detected_beacons.items():
                if beacon_name.lower() == search_name.lower():
                    return beacon_data
        
        # If no exact match or we're using a pattern, try pattern matching
        if self.target_beacon_pattern:
            pattern = self.target_beacon_pattern.lower()
            for beacon_name, beacon_data in self.detected_beacons.items():
                if pattern in beacon_name.lower():
                    return beacon_data
        
        return None
    
    def _on_device_detected(self, device: BLEDevice, advertisement_data: AdvertisementData):
        """
        Callback for device detection during scanning.
        
        Args:
            device: BLE device information
            advertisement_data: Advertisement data from the device
        """
        # Get device name (try different sources)
        name = device.name or advertisement_data.local_name or "Unknown"
        
        # Skip devices without a name unless they look interesting
        if name == "Unknown" and not self._is_interesting_device(device, advertisement_data):
            return
        
        # Handle duplicate names by appending MAC address if a beacon with this name already exists
        # and has a different MAC address
        unique_name = name
        if name in self.detected_beacons and self.detected_beacons[name]['address'] != device.address:
            # Create a unique name by appending the last 4 chars of the MAC address
            short_mac = device.address.replace(':', '')[-4:]
            unique_name = f"{name}_{short_mac}"
            self.logger.debug(f"Found duplicate name '{name}', using '{unique_name}' for tracking")
        
        # Store device data
        self.detected_beacons[unique_name] = {
            'address': device.address,
            'rssi': device.rssi,
            'name': name,  # Keep the original name
            'display_name': unique_name,  # For UI display
            'service_uuids': advertisement_data.service_uuids,
            'service_data': advertisement_data.service_data,
            'manufacturer_data': advertisement_data.manufacturer_data,
            'last_seen': time.time()
        }
        
        # Check if this is our target beacon - either by exact name or pattern
        is_target = False
        
        # Check exact name match
        if self.target_beacon_name:
            # Try original name first
            if name.lower() == self.target_beacon_name.lower():
                is_target = True
            # Then try unique name (for beacons we've already renamed)
            elif unique_name.lower() == self.target_beacon_name.lower():
                is_target = True
        
        # Check pattern match if we have a pattern and haven't found an exact match
        if not is_target and self.target_beacon_pattern:
            pattern = self.target_beacon_pattern.lower()
            if pattern in name.lower() or pattern in unique_name.lower():
                is_target = True
        
        if is_target:
            self.target_beacon_data = self.detected_beacons[unique_name]
            self.current_rssi = device.rssi
            self.rssi_readings.append(device.rssi)
            
            # Keep a moving window of readings
            if len(self.rssi_readings) > self.config['rssi_samples'] * 2:
                self.rssi_readings = self.rssi_readings[-self.config['rssi_samples']:]
            
            # Log the detection if we're actively tracking
            if self.tracking_state in [TrackingState.TRACKING, TrackingState.APPROACHING]:
                self.logger.debug(f"Target beacon detected: {unique_name} with RSSI: {device.rssi} dBm")
            
            # Publish event
            self._publish_event('beacon_detected', {
                'name': unique_name,
                'rssi': device.rssi,
                'address': device.address
            })
    
    def _is_interesting_device(self, device: BLEDevice, advertisement_data: AdvertisementData) -> bool:
        """
        Determine if a device is interesting enough to track even without a name.
        
        Args:
            device: BLE device information
            advertisement_data: Advertisement data from the device
            
        Returns:
            True if the device is interesting, False otherwise
        """
        # If it has service UUIDs or manufacturer data, it might be interesting
        if advertisement_data.service_uuids or advertisement_data.manufacturer_data:
            return True
        
        # If it has a very strong signal, it might be interesting
        if device.rssi > -60:
            return True
        
        return False
    
    async def scan_for_beacons(self, duration: float = 5.0) -> Dict[str, Dict]:
        """
        Scan for nearby BLE beacons.
        
        Args:
            duration: Scan duration in seconds
            
        Returns:
            Dictionary of detected beacons
        """
        with self.scan_lock:
            previous_state = self.tracking_state
            self.tracking_state = TrackingState.SCANNING
            self.detected_beacons = {}
            
            if self.scanner is None:
                self.scanner = BleakScanner()
                self.scanner.register_detection_callback(self._on_device_detected)
            
            self._publish_event('scan_start', {'duration': duration})
            self.logger.info(f"Scanning for beacons for {duration} seconds...")
            
            await self.scanner.start()
            await asyncio.sleep(duration)
            await self.scanner.stop()
            
            self.tracking_state = previous_state
            
            # Log found beacons
            beacon_count = len(self.detected_beacons)
            self.logger.info(f"Scan complete. Found {beacon_count} beacons.")
            for name, data in self.detected_beacons.items():
                self.logger.info(f"  - {name}: RSSI {data['rssi']} dBm")
            
            self._publish_event('scan_complete', {
                'count': beacon_count,
                'beacons': list(self.detected_beacons.keys())
            })
            
            return self.detected_beacons
    
    async def get_filtered_rssi(self) -> Optional[float]:
        """
        Get a filtered RSSI reading for the target beacon.
        
        Returns:
            Filtered RSSI value or None if no readings available
        """
        if not self.rssi_readings:
            return None
            
        # Apply median filtering to reduce noise
        readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
        
        # Remove outliers
        if len(readings) >= 5:
            readings.sort()
            # Remove top and bottom 20%
            cutoff = max(1, len(readings) // 5)
            readings = readings[cutoff:-cutoff]
        
        # Return median
        return statistics.median(readings) if readings else None
    
    def _start_continuous_scanning(self):
        """Start continuous BLE scanning in the background."""
        if self.scanning_task and not self.scanning_task.done():
            # Already scanning
            return
        
        self.scanning_task = asyncio.create_task(self._continuous_scan_loop())
        self.logger.debug("Started continuous BLE scanning")
    
    async def _continuous_scan_loop(self):
        """Background task for continuous BLE scanning."""
        try:
            while self.running and self.tracking_state in [TrackingState.TRACKING, TrackingState.APPROACHING, TrackingState.CALIBRATING]:
                if self.paused:
                    await asyncio.sleep(0.5)
                    continue
                
                with self.scan_lock:
                    # Start a scan
                    await self.scanner.start()
                    await asyncio.sleep(self.config['scan_interval'])
                    await self.scanner.stop()
                
                # Small pause before the next scan
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            self.logger.debug("Continuous scan loop cancelled")
            raise
        except Exception as e:
            self.logger.error(f"Error in continuous scan loop: {e}")
            if self.scanner:
                await self.scanner.stop()
    
    async def start_tracking(self):
        """
        Start continuous tracking of the target beacon.
        Must have set a target beacon name or pattern first.
        
        Returns:
            Success flag
        """
        if not self.target_beacon_name and not self.target_beacon_pattern:
            self.logger.error("No target beacon specified. Use set_target_beacon() or set_target_beacon_pattern() first.")
            return False
        
        if self.tracking_state != TrackingState.IDLE:
            self.logger.warning(f"Cannot start tracking while in {self.tracking_state.value} state")
            return False
        
        self.tracking_state = TrackingState.TRACKING
        self.is_tracking = True
        
        self._publish_event('tracking_start', {
            'target': self.target_beacon_name or f"pattern:{self.target_beacon_pattern}"
        })
        
        self.logger.info(f"Starting to track beacon: {self.target_beacon_name or self.target_beacon_pattern}")
        
        # Start continuous scanning in the background
        self._start_continuous_scanning()
        
        # Wait a moment to collect some initial readings
        await asyncio.sleep(2 * self.config['scan_interval'])
        
        # Start tracking task
        self.tracking_task = asyncio.create_task(self._tracking_loop())
        
        return True
    
    async def _tracking_loop(self):
        """Background task for continuous beacon tracking."""
        try:
            while self.running and self.is_tracking:
                if self.paused:
                    await asyncio.sleep(0.5)
                    continue
                
                # Check for too many errors
                if self.tracking_errors >= self.config['max_tracking_errors']:
                    self.logger.error(f"Too many tracking errors ({self.tracking_errors}), recalibrating")
                    await self.calibrate(force=True)
                    self.tracking_errors = 0
                
                # Track beacon
                if self.tracking_state == TrackingState.TRACKING:
                    await self._track_beacon_once()
                elif self.tracking_state == TrackingState.APPROACHING:
                    await self._approach_beacon_once()
                
                # Wait for next tracking iteration
                await asyncio.sleep(self.config['tracking_interval'])
                
        except asyncio.CancelledError:
            self.logger.debug("Tracking loop cancelled")
            raise
        except Exception as e:
            self.logger.error(f"Error in tracking loop: {e}")
            self.tracking_errors += 1
    
    async def stop_tracking(self):
        """
        Stop tracking the target beacon.
        
        Returns:
            Success flag
        """
        if self.tracking_state not in [TrackingState.TRACKING, TrackingState.APPROACHING]:
            return True
        
        self.is_tracking = False
        
        # Stop the tracking task
        if self.tracking_task and not self.tracking_task.done():
            self.tracking_task.cancel()
            try:
                await self.tracking_task
            except asyncio.CancelledError:
                pass
            self.tracking_task = None
        
        # Stop the scanning task
        if self.scanning_task and not self.scanning_task.done():
            self.scanning_task.cancel()
            try:
                await self.scanning_task
            except asyncio.CancelledError:
                pass
            self.scanning_task = None
        
        # Stop the scanner
        if self.scanner:
            await self.scanner.stop()
        
        # Stop movement
        self.motion_controller.stop(high_priority=True)
        
        self.tracking_state = TrackingState.IDLE
        
        self._publish_event('tracking_stop', {})
        self.logger.info("Stopped tracking beacon")
        
        return True
    
    async def _track_beacon_once(self):
        """Perform one iteration of beacon tracking."""
        # Check if we have recent target beacon data
        if not self.target_beacon_data or time.time() - self.target_beacon_data.get('last_seen', 0) > 5:
            self.logger.warning("Target beacon not detected recently")
            self.motion_controller.stop()
            
            # Search for the beacon
            await self._search_for_beacon()
            self.tracking_errors += 1
            return
        
        # Calculate the average RSSI
        avg_rssi = await self.get_filtered_rssi()
        
        if avg_rssi is None:
            self.logger.warning("No RSSI readings available")
            self.motion_controller.stop()
            await asyncio.sleep(1)
            self.tracking_errors += 1
            return
        
        # Decide on movement based on RSSI
        await self._plan_movement_based_on_rssi(avg_rssi)
        
        # Reset error counter on success
        self.tracking_errors = 0
    
    async def approach_beacon(self):
        """
        Begin approaching the target beacon.
        Uses RSSI readings to navigate toward the beacon.
        
        Returns:
            Success flag
        """
        if not await self.start_tracking():
            return False
        
        self.tracking_state = TrackingState.APPROACHING
        
        self._publish_event('approaching_start', {
            'target': self.target_beacon_name or f"pattern:{self.target_beacon_pattern}"
        })
        
        self.logger.info(f"Beginning approach to beacon: {self.target_beacon_name or self.target_beacon_pattern}")
        
        return True
    
    async def _approach_beacon_once(self):
        """Perform one iteration of beacon approach."""
        # This is similar to _track_beacon_once but more aggressive in movement
        
        # Check if we have recent target beacon data
        if not self.target_beacon_data or time.time() - self.target_beacon_data.get('last_seen', 0) > 5:
            self.logger.warning("Target beacon not detected recently during approach")
            self.motion_controller.stop()
            
            # Search for the beacon
            await self._search_for_beacon()
            self.tracking_errors += 1
            return
        
        # Calculate the average RSSI
        avg_rssi = await self.get_filtered_rssi()
        
        if avg_rssi is None:
            self.logger.warning("No RSSI readings available during approach")
            self.motion_controller.stop()
            await asyncio.sleep(1)
            self.tracking_errors += 1
            return
        
        # If signal is very strong, we're close
        if avg_rssi > -45:  # Very close threshold
            self.logger.info(f"Very close to beacon (RSSI: {avg_rssi} dBm)")
            self.motion_controller.stop()
            
            self._publish_event('beacon_reached', {
                'rssi': avg_rssi,
                'target': self.target_beacon_name or f"pattern:{self.target_beacon_pattern}"
            })
            
            # Transition back to tracking state to maintain position
            self.tracking_state = TrackingState.TRACKING
            return
        
        # Decide on movement based on RSSI - more aggressive for approach
        await self._plan_movement_based_on_rssi(avg_rssi, aggressive=True)
        
        # Reset error counter on success
        self.tracking_errors = 0
    
    async def _plan_movement_based_on_rssi(self, current_rssi: float, aggressive=False):
        """
        Improved movement planning based on RSSI values.
        Uses dynamic movement durations and improved decision making.
        
        Args:
            current_rssi: Current average RSSI value
            aggressive: Whether to use more aggressive movement (for approach mode)
        """
        # If signal is very strong, we're close
        if current_rssi > -50 and not aggressive:  # Close threshold
            self.logger.info(f"Close to beacon (RSSI: {current_rssi} dBm)")
            self.motion_controller.stop()
            await asyncio.sleep(1.0)  # Pause when close
            return
        
        # If we have calibration data, use it
        if self.calibration_data.get('best_direction'):
            best_direction = self.calibration_data['best_direction']
            
            # Determine movement duration based on signal strength
            # Stronger signal = shorter movement time
            if current_rssi > -60:
                duration = 0.8  # Short movements when close
            elif current_rssi > -70:
                duration = 1.2  # Medium movements
            else:
                duration = 1.8  # Longer movements when far
            
            # Adjust duration for aggressive approach
            if aggressive:
                duration *= 1.5
            
            # Move in the best direction based on calibration
            await self._continuous_movement(best_direction, duration)
            
            self.logger.info(f"Moving {best_direction.lower()} for {duration}s, RSSI: {current_rssi} dBm")
        else:
            # No calibration data, use simpler approach
            # Adjust threshold based on signal strength
            adjusted_threshold = max(self.config['rssi_min_threshold'], current_rssi - 10)
            
            if current_rssi < adjusted_threshold:
                await self._perform_signal_search(aggressive)
            else:
                # Move forward with dynamic duration
                duration = 1.5 if current_rssi < -70 else 1.0
                
                # Adjust for aggressive approach
                if aggressive:
                    duration *= 1.5
                    
                self.motion_controller.forward(self.config['movement_speed'])
                await asyncio.sleep(duration)
                self.motion_controller.stop()
                self.logger.info(f"Moving forward for {duration}s, RSSI: {current_rssi} dBm")
        
        # Wait for stable readings
        await asyncio.sleep(0.3)  # Short pause for readings to stabilize
    
    async def _perform_signal_search(self, aggressive=False):
        """
        Optimized search pattern to find better signal.
        Tests all directions and moves in the best one.
        
        Args:
            aggressive: Whether to use more aggressive movement
        """
        # Get the direction enum or fallback to string-based directions
        try:
            direction_enum = self.motion_controller.current_direction.__class__
            directions = [
                (direction_enum.FORWARD, "forward"),
                (direction_enum.LEFT, "left"),
                (direction_enum.RIGHT, "right"),
                # Only include BACKWARD if really needed
                (direction_enum.BACKWARD, "backward")
            ]
        except (AttributeError, TypeError):
            # Fallback to string-based directions
            directions = [
                ("FORWARD", "forward"),
                ("LEFT", "left"),
                ("RIGHT", "right"),
                ("BACKWARD", "backward")
            ]
        
        best_rssi = float('-inf')
        best_dir = None
        
        # Save initial readings to compare against
        initial_readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
        initial_avg = statistics.mean(initial_readings) if initial_readings else None
        
        test_time = 0.8 if not aggressive else 1.0
        
        for direction, name in directions:
            # Move in this direction
            await self._move_in_direction(direction, self.config['movement_speed'])
            
            self.logger.debug(f"Testing direction: {name}")
            
            # Move for a short time
            await asyncio.sleep(test_time)
            self.motion_controller.stop()
            
            # Wait for stable readings
            await asyncio.sleep(0.3)
            
            # Take RSSI readings
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
            if readings:
                avg_rssi = statistics.mean(readings)
                self.logger.debug(f"Direction {name}: avg RSSI = {avg_rssi} dBm")
                
                # Track readings by direction
                self.rssi_by_direction[direction if isinstance(direction, str) else direction.name].append(avg_rssi)
                
                # Find best direction with significant improvement
                if avg_rssi > best_rssi:
                    # Require significant improvement for backward direction
                    if (isinstance(direction, str) and direction == "BACKWARD") or \
                       (not isinstance(direction, str) and direction == direction_enum.BACKWARD):
                        if initial_avg is not None:
                            # Only choose backward if it's significantly better
                            if avg_rssi > initial_avg + 5:
                                best_rssi = avg_rssi
                                best_dir = direction
                    else:
                        best_rssi = avg_rssi
                        best_dir = direction
        
        # Move in the best direction found
        if best_dir:
            dir_name = best_dir if isinstance(best_dir, str) else best_dir.name
            self.logger.info(f"Best direction: {dir_name.lower()} with RSSI: {best_rssi} dBm")
            
            # Calculate movement duration based on improvement
            improvement = best_rssi - (initial_avg or -100)
            duration = min(2.0, max(1.0, 1.0 + improvement/10))
            
            # Adjust for aggressive approach
            if aggressive:
                duration *= 1.5
                
            # Move longer in the best direction
            await self._move_in_direction(best_dir, self.config['movement_speed'])
            await asyncio.sleep(duration)
            self.motion_controller.stop()
        else:
            self.logger.warning("No good direction found")
            # Try a spiral search pattern
            await self._spiral_search()
    
    async def _move_in_direction(self, direction, speed):
        """
        Move in the specified direction.
        Handles both enum-based and string-based directions.
        
        Args:
            direction: Direction to move (enum or string)
            speed: Movement speed
        """
        # Check if direction is a string or enum
        if isinstance(direction, str):
            # String-based direction
            if direction == "FORWARD":
                self.motion_controller.forward(speed)
            elif direction == "BACKWARD":
                self.motion_controller.backward(speed)
            elif direction == "LEFT":
                self.motion_controller.turn_left(speed)
            elif direction == "RIGHT":
                self.motion_controller.turn_right(speed)
            else:
                self.logger.warning(f"Unknown direction: {direction}")
                self.motion_controller.stop()
        else:
            # Enum-based direction
            try:
                direction_enum = self.motion_controller.current_direction.__class__
                if direction == direction_enum.FORWARD:
                    self.motion_controller.forward(speed)
                elif direction == direction_enum.BACKWARD:
                    self.motion_controller.backward(speed)
                elif direction == direction_enum.LEFT:
                    self.motion_controller.turn_left(speed)
                elif direction == direction_enum.RIGHT:
                    self.motion_controller.turn_right(speed)
                else:
                    self.logger.warning(f"Unknown direction enum: {direction}")
                    self.motion_controller.stop()
            except (AttributeError, TypeError) as e:
                self.logger.error(f"Error moving in direction {direction}: {e}")
                self.motion_controller.stop()
    
    async def _spiral_search(self, max_steps=None):
        """
        Perform a spiral search pattern to find the beacon.
        
        Args:
            max_steps: Maximum number of steps in the spiral (default: from config)
        """
        if max_steps is None:
            max_steps = self.config['search_max_spiral_steps']
            
        self.logger.info("Performing spiral search for beacon")
        
        # Spiral pattern: Move forward, turn, move forward longer, turn...
        step_size = 1.0  # Base step time in seconds
        
        for i in range(1, max_steps + 1):
            # Move forward
            self.motion_controller.forward(self.config['movement_speed'])
            await asyncio.sleep(step_size * i)
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
            
            # Check if we found the beacon with good signal
            if self.target_beacon_data and time.time() - self.target_beacon_data.get('last_seen', 0) < 1:
                avg_rssi = await self.get_filtered_rssi()
                if avg_rssi and avg_rssi > self.config['rssi_min_threshold']:
                    self.logger.info(f"Found beacon during spiral search with RSSI: {avg_rssi}")
                    return True
            
            # Turn right
            self.motion_controller.turn_right(self.config['movement_speed'])
            await asyncio.sleep(0.5)  # 90 degree turn
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
            
            # Move forward again
            self.motion_controller.forward(self.config['movement_speed'])
            await asyncio.sleep(step_size * i)
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
            
            # Check if we found the beacon
            if self.target_beacon_data and time.time() - self.target_beacon_data.get('last_seen', 0) < 1:
                avg_rssi = await self.get_filtered_rssi()
                if avg_rssi and avg_rssi > self.config['rssi_min_threshold']:
                    self.logger.info(f"Found beacon during spiral search with RSSI: {avg_rssi}")
                    return True
            
            # Turn right
            self.motion_controller.turn_right(self.config['movement_speed'])
            await asyncio.sleep(0.5)  # 90 degree turn
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
        
        self.logger.warning("Spiral search completed without finding the beacon")
        return False
    
    async def _search_for_beacon(self):
        """
        Search for the beacon if it's lost.
        
        Returns:
            True if beacon was found, False otherwise
        """
        self.logger.info("Searching for lost beacon")
        
        rotation_steps = self.config['search_rotation_steps']
        
        # First, do a 360-degree scan
        for _ in range(rotation_steps):  # rotation_steps x (360/rotation_steps) degrees = 360 degrees
            self.motion_controller.turn_right(self.config['movement_speed'])
            await asyncio.sleep(360 / (90 * rotation_steps))  # Assuming 90 degrees takes 1 second
            self.motion_controller.stop()
            await asyncio.sleep(1.0)  # Pause to scan
            
            # Check if we found the beacon
            if self.target_beacon_data and time.time() - self.target_beacon_data.get('last_seen', 0) < 2:
                avg_rssi = await self.get_filtered_rssi()
                self.logger.info(f"Found beacon during rotation search with RSSI: {avg_rssi}")
                return True
        
        # If not found, try a spiral search
        return await self._spiral_search()
    
    async def continuous_movement(self, direction_name, duration=1.5):
        """
        Move continuously in the specified direction for better momentum.
        
        Args:
            direction_name: Direction to move in (FORWARD, BACKWARD, etc.)
            duration: Time to move in seconds
        """
        await self._continuous_movement(direction_name, duration)
        
    async def _continuous_movement(self, direction_name, duration=1.5):
        """
        Internal implementation of continuous movement.
        
        Args:
            direction_name: Direction to move in (string)
            duration: Time to move in seconds
        """
        # Handle both enum names and string directions
        if isinstance(direction_name, Enum):
            direction_name = direction_name.name
            
        direction_name = direction_name.upper()
        
        # Execute the move with the specific direction
        if direction_name == "FORWARD":
            self.motion_controller.forward(self.config['movement_speed'])
        elif direction_name == "BACKWARD":
            self.motion_controller.backward(self.config['movement_speed'])
        elif direction_name == "LEFT":
            self.motion_controller.turn_left(self.config['movement_speed'])
        elif direction_name == "RIGHT":
            self.motion_controller.turn_right(self.config['movement_speed'])
        else:
            self.logger.warning(f"Unknown direction: {direction_name}")
            return
        
        # Move for the specified duration
        await asyncio.sleep(duration)
        
        # Stop
        self.motion_controller.stop()
    
    async def calibrate(self, force=False, callback=None):
        """
        Perform a calibration routine to optimize beacon tracking.
        This includes measuring RSSI at different rotations and distances.
        
        Args:
            force: Force calibration even if previous calibration is valid
            callback: Optional callback function for progress updates
            
        Returns:
            Calibration results dictionary or None on failure
        """
        # Check if we need to calibrate
        if not force and await self._check_calibration_validity():
            self.logger.info("Using existing valid calibration")
            return self.calibration_data
            
        if not self.target_beacon_name and not self.target_beacon_pattern:
            self.logger.error("No target beacon specified. Use set_target_beacon() or set_target_beacon_pattern() first.")
            return None
        
        prev_state = self.tracking_state
        self.tracking_state = TrackingState.CALIBRATING
        self.calibration_phase = CalibrationPhase.INIT
        
        self._publish_event('calibration_start', {
            'target': self.target_beacon_name or f"pattern:{self.target_beacon_pattern}"
        })
        
        self.logger.info(f"Starting calibration for beacon: {self.target_beacon_name or self.target_beacon_pattern}")
        
        # Start continuous scanning
        self._start_continuous_scanning()
        
        # Initialize calibration data
        self.calibration_data = {
            'rotation_map': {},
            'distance_map': {},
            'best_direction': None,
            'best_angle': None,
            'signal_peak': None,
            'timestamp': time.time(),
            'environment_hash': await self._get_environment_hash()
        }
        
        # Reset RSSI readings
        self.rssi_readings = []
        
        try:
            # Wait for some initial readings
            await asyncio.sleep(1.0)
            
            # Phase 1: Rotational calibration
            if callback:
                callback("Starting rotational calibration")
            
            self.calibration_phase = CalibrationPhase.ROTATE_READINGS
            await self._calibrate_rotation()
            
            # Phase 2: Distance calibration
            if callback:
                callback("Starting distance calibration")
            
            self.calibration_phase = CalibrationPhase.DISTANCE_READINGS
            await self._calibrate_distance()
            
            # Phase 3: Analysis
            if callback:
                callback("Analyzing calibration data")
            
            self.calibration_phase = CalibrationPhase.ANALYSIS
            await self._analyze_calibration()
            
            # Complete
            self.calibration_phase = CalibrationPhase.COMPLETE
            
            if callback:
                callback("Calibration complete")
            
            self.logger.info("Calibration completed successfully")
            self.logger.info(f"Best direction: {self.calibration_data['best_direction']}")
            
            self._publish_event('calibration_complete', {
                'best_direction': self.calibration_data['best_direction'],
                'best_angle': self.calibration_data['best_angle'],
                'signal_peak': self.calibration_data['signal_peak']
            })
            
            return self.calibration_data
            
        except Exception as e:
            self.logger.error(f"Error during calibration: {e}")
            if callback:
                callback(f"Calibration error: {str(e)}")
                
            self._publish_event('calibration_error', {'error': str(e)})
            return None
        
        finally:
            # Restore previous state
            self.tracking_state = prev_state
            
            # Stop continuous scanning if we weren't tracking before
            if prev_state not in [TrackingState.TRACKING, TrackingState.APPROACHING]:
                if self.scanning_task:
                    self.scanning_task.cancel()
                    try:
                        await self.scanning_task
                    except asyncio.CancelledError:
                        pass
                    self.scanning_task = None
                
                if self.scanner:
                    await self.scanner.stop()
    
    async def _get_environment_hash(self):
        """
        Generate a hash representing the current environment conditions.
        
        Returns:
            Environment hash string
        """
        try:
            # Get all beacon data
            beacon_data = str(sorted([(k, v['rssi']) for k, v in self.detected_beacons.items()]))
            
            # Combine data for hash
            env_data = beacon_data
            
            # Generate hash
            return hashlib.md5(env_data.encode()).hexdigest()
            
        except Exception as e:
            self.logger.warning(f"Error generating environment hash: {e}")
            # Fallback to timestamp-based hash
            return hashlib.md5(str(time.time()).encode()).hexdigest()
    
    async def _check_calibration_validity(self):
        """
        Check if the current calibration is still valid.
        
        Returns:
            Validity flag
        """
        if not self.calibration_data:
            return False
            
        # Check if calibration is too old
        max_age = self.config['calibration_max_age']
        if time.time() - self.calibration_data.get('timestamp', 0) > max_age:
            return False
            
        # Check if environment has changed significantly
        current_env_hash = await self._get_environment_hash()
        return current_env_hash == self.calibration_data.get('environment_hash')
    
    async def _calibrate_rotation(self):
        """
        Perform rotational calibration.
        Measures RSSI while rotating in place.
        
        Returns:
            Dictionary of angle->RSSI mappings
        """
        self.logger.info("Starting rotational calibration")
        
        min_steps = self.config['rotation_min_steps']
        max_steps = self.config['rotation_max_steps']
        rotation_map = {}
        degrees_per_step = 360 / min_steps
        
        # Make sure we start from a known position
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        for i in range(min_steps):
            current_degrees = i * degrees_per_step
            self.logger.debug(f"Rotating to {current_degrees} degrees")
            
            # Rotate to the desired position
            self.motion_controller.turn_right(self.config['rotation_speed'])
            await asyncio.sleep(degrees_per_step / 90)  # Assuming 90 degrees takes about 1 second
            self.motion_controller.stop()
            
            # Wait for stable readings
            await asyncio.sleep(1.0)
            
            # Take RSSI readings at this position
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
            if readings:
                avg_rssi = statistics.mean(readings)
                rotation_map[current_degrees] = avg_rssi
                self.logger.debug(f"Position {current_degrees}°: RSSI = {avg_rssi} dBm")
                
                self._publish_event('calibration_update', {
                    'phase': 'rotation',
                    'angle': current_degrees,
                    'rssi': avg_rssi
                })
        
        # Add additional samples in areas with high gradient
        if min_steps < max_steps and len(rotation_map) >= 3:
            angles = sorted(rotation_map.keys())
            additional_points = []
            
            for i in range(len(angles)):
                current = angles[i]
                next_idx = (i + 1) % len(angles)
                next_angle = angles[next_idx]
                
                # Handle wrapping around 360
                if next_angle < current:
                    next_angle += 360
                
                # Calculate gradient
                gradient = abs(rotation_map[angles[next_idx] % 360] - rotation_map[current])
                if gradient > self.config['rssi_change_threshold']:
                    # Add a point in the middle
                    midpoint = (current + next_angle) / 2 % 360
                    additional_points.append(midpoint)
            
            # Sample additional points in high-gradient areas
            for angle in additional_points[:max_steps - min_steps]:
                self.logger.debug(f"Sampling additional angle: {angle} degrees")
                
                # Determine which way to rotate
                current_pos = (min_steps - 1) * degrees_per_step
                if angle < current_pos:
                    diff = current_pos - angle
                    if diff > 180:
                        # Shorter to go right
                        self.motion_controller.turn_right(self.config['rotation_speed'])
                        await asyncio.sleep((360 - diff) / 90)
                    else:
                        # Shorter to go left
                        self.motion_controller.turn_left(self.config['rotation_speed'])
                        await asyncio.sleep(diff / 90)
                else:
                    diff = angle - current_pos
                    if diff > 180:
                        # Shorter to go left
                        self.motion_controller.turn_left(self.config['rotation_speed'])
                        await asyncio.sleep((360 - diff) / 90)
                    else:
                        # Shorter to go right
                        self.motion_controller.turn_right(self.config['rotation_speed'])
                        await asyncio.sleep(diff / 90)
                
                self.motion_controller.stop()
                await asyncio.sleep(1.0)
                
                # Take RSSI readings
                readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
                if readings:
                    avg_rssi = statistics.mean(readings)
                    rotation_map[angle] = avg_rssi
                    self.logger.debug(f"Position {angle}°: RSSI = {avg_rssi} dBm")
                    
                    self._publish_event('calibration_update', {
                        'phase': 'rotation',
                        'angle': angle,
                        'rssi': avg_rssi
                    })
        
        # Store rotation data
        self.calibration_data['rotation_map'] = rotation_map
        
        # Return to the position with the strongest signal
        if rotation_map:
            best_degrees = max(rotation_map.items(), key=lambda x: x[1])[0]
            self.logger.info(f"Best signal at {best_degrees} degrees, RSSI: {rotation_map[best_degrees]} dBm")
            
            # Store best angle
            self.calibration_data['best_angle'] = best_degrees
            
            # Calculate how much to rotate to get back to the best position
            current_degrees = (min_steps - 1) * degrees_per_step  # We're at the last position
            degrees_to_turn = (best_degrees - current_degrees) % 360
            
            if degrees_to_turn > 180:
                # Shorter to turn left
                self.motion_controller.turn_left(self.config['rotation_speed'])
                await asyncio.sleep((360 - degrees_to_turn) / 90)
            else:
                # Shorter to turn right
                self.motion_controller.turn_right(self.config['rotation_speed'])
                await asyncio.sleep(degrees_to_turn / 90)
            
            self.motion_controller.stop()
            await asyncio.sleep(1.0)
            
        return rotation_map
    
    async def _calibrate_distance(self):
        """
        Perform distance calibration.
        Measures RSSI at different distances from the current position.
        
        Returns:
            Dictionary of distance->RSSI mappings
        """
        self.logger.info("Starting distance calibration")
        
        max_steps = self.config['distance_max_steps']
        step_time = self.config['distance_step_time']
        distance_map = {}
        
        # Make sure we start from a stopped position
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        # First, get current position reading
        readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
        if readings:
            avg_rssi = statistics.mean(readings)
            distance_map[0] = avg_rssi
            self.logger.debug(f"Base position: RSSI = {avg_rssi} dBm")
            
            self._publish_event('calibration_update', {
                'phase': 'distance',
                'distance': 0,
                'rssi': avg_rssi
            })
        
        # Test moving forward
        for i in range(1, max_steps + 1):
            # Move forward
            self.motion_controller.forward(self.config['movement_speed'])
            await asyncio.sleep(step_time)
            self.motion_controller.stop()
            
            # Wait for stable readings
            await asyncio.sleep(1.0)
            
            # Take RSSI readings at this position
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
            if readings:
                avg_rssi = statistics.mean(readings)
                distance_map[i] = avg_rssi
                self.logger.debug(f"Forward step {i}: RSSI = {avg_rssi} dBm")
                
                self._publish_event('calibration_update', {
                    'phase': 'distance',
                    'distance': i,
                    'rssi': avg_rssi
                })
                
                # If signal decreases significantly, stop moving forward
                if distance_map[0] - avg_rssi > self.config['rssi_change_threshold'] * 2:
                    self.logger.info(f"Signal decreased significantly after moving forward {i} steps, stopping")
                    break
        
        # Return to the base position
        self.motion_controller.backward(self.config['movement_speed'])
        await asyncio.sleep(step_time * max_steps)  # Go back enough to cover our steps
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        # Test moving backward
        for i in range(1, max_steps + 1):
            # Move backward
            self.motion_controller.backward(self.config['movement_speed'])
            await asyncio.sleep(step_time)
            self.motion_controller.stop()
            
            # Wait for stable readings
            await asyncio.sleep(1.0)
            
            # Take RSSI readings at this position
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.config['rssi_samples']):]
            if readings:
                avg_rssi = statistics.mean(readings)
                distance_map[-i] = avg_rssi
                self.logger.debug(f"Backward step {i}: RSSI = {avg_rssi} dBm")
                
                self._publish_event('calibration_update', {
                    'phase': 'distance',
                    'distance': -i,
                    'rssi': avg_rssi
                })
                
                # If signal decreases significantly, stop moving backward
                if distance_map[0] - avg_rssi > self.config['rssi_change_threshold'] * 2:
                    self.logger.info(f"Signal decreased significantly after moving backward {i} steps, stopping")
                    break
        
        # Return to the base position
        self.motion_controller.forward(self.config['movement_speed'])
        await asyncio.sleep(step_time * max_steps)  # Go forward enough to cover our steps
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        # Store distance data
        self.calibration_data['distance_map'] = distance_map
        
        return distance_map
    
    async def _analyze_calibration(self):
        """
        Analyze the calibration data to determine optimal movement strategies.
        
        Returns:
            Calibration analysis results
        """
        self.logger.info("Analyzing calibration data")
        
        # Analyze rotation data
        rotation_map = self.calibration_data['rotation_map']
        if rotation_map:
            # Find the direction with the strongest signal
            best_degrees, best_rssi = max(rotation_map.items(), key=lambda x: x[1])
            self.logger.info(f"Best rotation: {best_degrees} degrees with RSSI: {best_rssi} dBm")
            
            # Store the peak signal strength and best angle
            self.calibration_data['signal_peak'] = best_rssi
            self.calibration_data['best_angle'] = best_degrees
        
        # Analyze distance data
        distance_map = self.calibration_data['distance_map']
        if distance_map:
            # Find the best step direction
            best_step, best_rssi = max(distance_map.items(), key=lambda x: x[1])
            
            # Determine the best movement direction based on calibration data
            if best_step > 0:
                best_direction = "FORWARD"
            elif best_step < 0:
                best_direction = "BACKWARD"
            else:
                # If 0 is best, determine from rotation
                if rotation_map:
                    best_degrees = max(rotation_map.items(), key=lambda x: x[1])[0]
                    if 45 <= best_degrees <= 135:
                        best_direction = "RIGHT"
                    elif 225 <= best_degrees <= 315:
                        best_direction = "LEFT"
                    else:
                        best_direction = "FORWARD"
                else:
                    best_direction = "FORWARD"
            
            self.logger.info(f"Best movement direction: {best_direction}")
            self.calibration_data['best_direction'] = best_direction
            
            # Calculate signal model parameters
            if len(distance_map) >= 3:
                try:
                    # Calculate simple log-distance model if applicable
                    self.calibration_data['signal_model'] = self._calculate_signal_model(distance_map)
                except Exception as e:
                    self.logger.warning(f"Error calculating signal model: {e}")
            
        # Update timestamp and environment hash
        self.calibration_data['timestamp'] = time.time()
        self.calibration_data['environment_hash'] = await self._get_environment_hash()
        
        return self.calibration_data
    
    def _calculate_signal_model(self, distance_map):
        """
        Calculate signal propagation model from calibration data.
        
        Args:
            distance_map: Map of distance -> RSSI values
            
        Returns:
            Signal model parameters
        """
        # Simple model: Find peak signal and fall-off rate
        distances = list(distance_map.keys())
        rssis = list(distance_map.values())
        
        # Find distance with maximum RSSI
        max_idx = rssis.index(max(rssis))
        peak_distance = distances[max_idx]
        peak_rssi = rssis[max_idx]
        
        # Calculate decay rates in both directions from peak
        forward_decay = []
        backward_decay = []
        
        for d, r in distance_map.items():
            if d > peak_distance:
                # Forward of peak
                if d - peak_distance > 0:  # Avoid division by zero
                    decay = (peak_rssi - r) / (d - peak_distance)
                    forward_decay.append(decay)
            elif d < peak_distance:
                # Backward of peak
                if peak_distance - d > 0:  # Avoid division by zero
                    decay = (peak_rssi - r) / (peak_distance - d)
                    backward_decay.append(decay)
        
        # Calculate average decay rates
        avg_forward_decay = statistics.mean(forward_decay) if forward_decay else 0
        avg_backward_decay = statistics.mean(backward_decay) if backward_decay else 0
        
        return {
            'model_type': 'peak-decay',
            'peak_distance': peak_distance,
            'peak_rssi': peak_rssi,
            'forward_decay': avg_forward_decay,
            'backward_decay': avg_backward_decay
        }
    
    async def estimate_beacon_position(self):
        """
        Estimate the relative position of the target beacon.
        
        Returns:
            Dict with estimated position information or None on failure
        """
        if not self.target_beacon_data:
            self.logger.warning("No target beacon data available")
            return None
            
        # Check if we have calibration data
        if not self.calibration_data:
            self.logger.warning("No calibration data available")
            return None
            
        # Get filtered RSSI
        current_rssi = await self.get_filtered_rssi()
        if current_rssi is None:
            self.logger.warning("Could not get filtered RSSI for position estimation")
            return None
            
        # Use signal model if available
        if 'signal_model' in self.calibration_data:
            model = self.calibration_data['signal_model']
            
            if model['model_type'] == 'peak-decay':
                # Use the peak-decay model
                peak_rssi = model['peak_rssi']
                peak_distance = model['peak_distance']
                
                # Determine if we're in front of or behind the peak
                if current_rssi >= peak_rssi:
                    # At or past the peak, distance is approximately the peak distance
                    relative_distance = peak_distance
                elif current_rssi < peak_rssi:
                    # Before the peak, use the appropriate decay rate
                    rssi_diff = peak_rssi - current_rssi
                    
                    if rssi_diff > 0:
                        if current_rssi > peak_rssi - 10:  # Very close to peak
                            # We're likely near the peak, estimate conservatively
                            relative_distance = peak_distance
                        else:
                            # Determine if we're in front of or behind the peak
                            forward_decay = model.get('forward_decay', 1)
                            backward_decay = model.get('backward_decay', 1)
                            
                            # Use calibration data to determine position
                            best_direction = self.calibration_data.get('best_direction')
                            
                            if best_direction == "FORWARD":
                                # We need to move forward, so we're likely behind the peak
                                if backward_decay > 0:
                                    distance_behind = rssi_diff / backward_decay
                                    relative_distance = peak_distance - distance_behind
                                else:
                                    relative_distance = peak_distance
                            elif best_direction == "BACKWARD":
                                # We need to move backward, so we're likely past the peak
                                if forward_decay > 0:
                                    distance_past = rssi_diff / forward_decay
                                    relative_distance = peak_distance + distance_past
                                else:
                                    relative_distance = peak_distance
                            else:
                                # Not sure, estimate based on RSSI only
                                relative_distance = peak_distance * (current_rssi / peak_rssi)
                    else:
                        relative_distance = peak_distance
                
                # Get best angle from calibration
                best_angle = self.calibration_data.get('best_angle', 0)
                
                position_estimate = {
                    'distance': relative_distance,
                    'angle': best_angle,
                    'rssi': current_rssi,
                    'confidence': min(1.0, max(0.2, current_rssi / peak_rssi))
                }
                
                return position_estimate
        
        # Fallback to simpler model if no signal model available
        best_angle = self.calibration_data.get('best_angle', 0)
        best_rssi = self.calibration_data.get('signal_peak', -60)
        
        # Very rough distance estimate based on RSSI
        if current_rssi >= -50:
            distance_estimate = 1  # Very close
        elif current_rssi >= -65:
            distance_estimate = 2  # Close
        elif current_rssi >= -75:
            distance_estimate = 3  # Medium
        else:
            distance_estimate = 5  # Far
        
        position_estimate = {
            'distance': distance_estimate,
            'angle': best_angle,
            'rssi': current_rssi,
            'confidence': 0.5  # Medium confidence for this simple model
        }
        
        return position_estimate
    
    def get_state(self):
        """
        Get the current state of the beacon tracker.
        
        Returns:
            Dictionary with current state information
        """
        target_info = self.target_beacon_name
        if not target_info and self.target_beacon_pattern:
            target_info = f"pattern:{self.target_beacon_pattern}"
            
        # Get names of detected beacons for reporting
        beacon_names = list(self.detected_beacons.keys())
        
        # Get RSSI statistics
        avg_rssi = None
        if self.rssi_readings:
            avg_rssi = statistics.mean(self.rssi_readings)
        
        # Assemble state information
        state_data = {
            'state': self.tracking_state.value,
            'target_beacon': target_info,
            'target_beacon_data': self.target_beacon_data.get('display_name') if self.target_beacon_data else None,
            'rssi': self.current_rssi,
            'avg_rssi': avg_rssi,
            'detected_beacons_count': len(self.detected_beacons),
            'detected_beacons': beacon_names[:10],  # Limit to first 10 for display
            'is_tracking': self.is_tracking,
            'is_calibrated': self.calibration_phase == CalibrationPhase.COMPLETE,
            'calibration_phase': self.calibration_phase.value if hasattr(self, 'calibration_phase') else None,
            'best_direction': self.calibration_data.get('best_direction'),
            'best_angle': self.calibration_data.get('best_angle'),
            'tracking_errors': self.tracking_errors,
            'paused': self.paused,
            'timestamp': time.time()
        }
        
        return state_data
    
    async def reset(self):
        """
        Reset the tracker state.
        
        Returns:
            Success flag
        """
        # Stop tracking
        if self.is_tracking:
            await self.stop_tracking()
        
        # Stop the robot
        self.motion_controller.stop(high_priority=True)
        
        # Reset state variables
        self.tracking_state = TrackingState.IDLE
        self.calibration_phase = CalibrationPhase.INIT
        self.rssi_readings = []
        self.detected_beacons = {}
        self.target_beacon_data = None
        self.current_rssi = None
        self.tracking_errors = 0
        
        # Reset position tracking
        self.current_position = (0, 0)
        self.current_rotation = 0
        
        # Reset calibration data
        self.calibration_data = {
            'rotation_map': {},
            'distance_map': {},
            'best_direction': None,
            'best_angle': None,
            'signal_peak': None,
            'timestamp': 0,
            'environment_hash': None
        }
        
        self._publish_event('reset', {})
        self.logger.info("Beacon tracker reset")
        
        return True