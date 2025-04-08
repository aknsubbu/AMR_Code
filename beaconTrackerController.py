"""
Beacon Tracker Controller for locating and moving toward BLE beacons.
Integrates motion control and BLE scanning with RSSI-based navigation.
"""
import asyncio
import logging
import time
import threading
import statistics
from enum import Enum
from typing import Dict, List, Optional, Tuple, Callable

from bleak import BleakScanner
from bleak.backends.scanner import AdvertisementData
from bleak.backends.device import BLEDevice

logger = logging.getLogger(__name__)

class BeaconTrackerState(Enum):
    """Enum for tracker states."""
    IDLE = 'idle'
    CALIBRATING = 'calibrating'
    SCANNING = 'scanning'
    TRACKING = 'tracking'
    APPROACHING = 'approaching'

class CalibrationPhase(Enum):
    """Enum for calibration phases."""
    INIT = 'init'
    ROTATE_READINGS = 'rotate_readings'
    DISTANCE_READINGS = 'distance_readings'
    ANALYSIS = 'analysis'
    COMPLETE = 'complete'

class BeaconTrackerController:
    """
    Controller for tracking and approaching BLE beacons.
    Uses RSSI signal strength to navigate toward a specified beacon.
    """
    
    def __init__(self, motion_controller, 
                 target_beacon_name: str = None,
                 target_beacon_pattern: str = None,
                 scan_interval: float = 1.0,
                 rssi_samples: int = 5,
                 rssi_threshold: int = -80,
                 movement_speed: int = 100):
        """
        Initialize the beacon tracker controller.
        
        Args:
            motion_controller: MotionController instance for robot movement
            target_beacon_name: Name of the beacon to track (case insensitive, exact match)
            target_beacon_pattern: Pattern to match in beacon names (case insensitive, partial match)
            scan_interval: Interval between BLE scans in seconds
            rssi_samples: Number of RSSI samples to collect per position
            rssi_threshold: Minimum RSSI value to consider valid (-30 is very close, -90 is far)
            movement_speed: Default movement speed (0-255)
        """
        self.motion_controller = motion_controller
        self.target_beacon_name = target_beacon_name
        self.target_beacon_pattern = target_beacon_pattern
        self.scan_interval = scan_interval
        self.rssi_samples = rssi_samples
        self.rssi_threshold = rssi_threshold
        self.movement_speed = movement_speed
        
        # BLE scanner
        self.scanner = None
        self.scanning_task = None
        self.scan_lock = threading.Lock()
        
        # State tracking
        self.state = BeaconTrackerState.IDLE
        self.calibration_phase = CalibrationPhase.INIT
        self.detected_beacons = {}  # Map of beacon names to their data
        self.target_beacon_data = None
        self.current_rssi = None
        self.rssi_readings = []
        self.rssi_by_direction = {}  # Will be populated based on MovementDirection enum
        
        # Calibration data
        self.calibration_data = {
            'rotation_map': {},  # Maps degrees to RSSI values
            'distance_map': {},  # Maps distances to RSSI values
            'best_direction': None,
            'signal_peak': None,
        }
        
        # Control flags
        self.running = False
        self.paused = False
        
        logger.info("Beacon tracker controller initialized")
    
    async def start(self):
        """Start the beacon tracker."""
        if self.running:
            logger.warning("Beacon tracker already running")
            return
        
        self.running = True
        self.state = BeaconTrackerState.IDLE
        
        # Initialize BLE scanner
        self.scanner = BleakScanner()
        self.scanner.register_detection_callback(self._on_device_detected)
        
        # Initialize rssi_by_direction based on MovementDirection enum
        for direction in self.motion_controller.current_direction.__class__:
            self.rssi_by_direction[direction.name] = []
        
        logger.info("Beacon tracker started")
    
    async def stop(self):
        """Stop the beacon tracker and any ongoing operations."""
        if not self.running:
            return
        
        self.running = False
        
        # Stop any ongoing scanning
        if self.scanner:
            await self.scanner.stop()
        
        # Stop the robot
        self.motion_controller.stop(high_priority=True)
        
        self.state = BeaconTrackerState.IDLE
        logger.info("Beacon tracker stopped")
    
    async def pause(self):
        """Pause the tracker temporarily."""
        if not self.running or self.paused:
            return
        
        self.paused = True
        self.motion_controller.stop(high_priority=True)
        logger.info("Beacon tracker paused")
    
    async def resume(self):
        """Resume the tracker after pausing."""
        if not self.running or not self.paused:
            return
        
        self.paused = False
        logger.info("Beacon tracker resumed")
    
    def set_target_beacon(self, beacon_name: str):
        """
        Set the target beacon to track by exact name match.
        
        Args:
            beacon_name: Name of the beacon to track (case insensitive)
        """
        self.target_beacon_name = beacon_name
        self.target_beacon_pattern = None  # Clear any pattern when setting exact name
        self.target_beacon_data = None  # Reset any existing data
        logger.info(f"Target beacon set to: {beacon_name}")
        
    def set_target_beacon_pattern(self, pattern: str):
        """
        Set a pattern to match in beacon names.
        
        Args:
            pattern: String pattern to look for in beacon names (case insensitive)
        """
        self.target_beacon_pattern = pattern
        self.target_beacon_name = None  # Clear any exact name when setting pattern
        self.target_beacon_data = None  # Reset any existing data
        logger.info(f"Target beacon pattern set to: {pattern}")
        
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
    
    async def scan_for_beacons(self, duration: float = 5.0) -> Dict[str, Dict]:
        """
        Scan for nearby BLE beacons.
        
        Args:
            duration: Scan duration in seconds
            
        Returns:
            Dictionary of detected beacons
        """
        with self.scan_lock:
            self.state = BeaconTrackerState.SCANNING
            self.detected_beacons = {}
            
            if self.scanner is None:
                self.scanner = BleakScanner()
                self.scanner.register_detection_callback(self._on_device_detected)
            
            logger.info(f"Scanning for beacons for {duration} seconds...")
            
            await self.scanner.start()
            await asyncio.sleep(duration)
            await self.scanner.stop()
            
            self.state = BeaconTrackerState.IDLE
            
            # Log found beacons
            beacon_count = len(self.detected_beacons)
            logger.info(f"Scan complete. Found {beacon_count} beacons.")
            for name, data in self.detected_beacons.items():
                logger.info(f"  - {name}: RSSI {data['rssi']} dBm")
            
            return self.detected_beacons
    
    def _on_device_detected(self, device: BLEDevice, advertisement_data: AdvertisementData):
        """
        Callback for device detection during scanning.
        
        Args:
            device: BLE device information
            advertisement_data: Advertisement data from the device
        """
        # Get device name (try different sources)
        name = device.name or advertisement_data.local_name or "Unknown"
        
        # Skip devices without a name
        if name == "Unknown" and not self._is_interesting_device(device, advertisement_data):
            return
        
        # Handle duplicate names by appending MAC address if a beacon with this name already exists
        # and has a different MAC address
        unique_name = name
        if name in self.detected_beacons and self.detected_beacons[name]['address'] != device.address:
            # Create a unique name by appending the last 4 chars of the MAC address
            short_mac = device.address.replace(':', '')[-4:]
            unique_name = f"{name}_{short_mac}"
            logger.debug(f"Found duplicate name '{name}', using '{unique_name}' for tracking")
        
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
            if len(self.rssi_readings) > self.rssi_samples * 2:
                self.rssi_readings = self.rssi_readings[-self.rssi_samples:]
            
            # Log the detection if we're actively tracking
            if self.state in [BeaconTrackerState.TRACKING, BeaconTrackerState.APPROACHING]:
                logger.info(f"Target beacon detected: {unique_name} with RSSI: {device.rssi} dBm")
    
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
    
    async def start_tracking(self):
        """
        Start continuous tracking of the target beacon.
        Must have set a target beacon name or pattern first.
        """
        if not self.target_beacon_name and not self.target_beacon_pattern:
            logger.error("No target beacon specified. Use set_target_beacon() or set_target_beacon_pattern() first.")
            return False
        
        if self.state != BeaconTrackerState.IDLE:
            logger.warning(f"Cannot start tracking while in {self.state.value} state")
            return False
        
        self.state = BeaconTrackerState.TRACKING
        logger.info(f"Starting to track beacon: {self.target_beacon_name}")
        
        # Start continuous scanning in the background
        self._start_continuous_scanning()
        
        # Wait a moment to collect some initial readings
        await asyncio.sleep(2 * self.scan_interval)
        
        return True
    
    async def stop_tracking(self):
        """Stop tracking the target beacon."""
        if self.state not in [BeaconTrackerState.TRACKING, BeaconTrackerState.APPROACHING]:
            return
        
        # Stop the scanning task
        if self.scanning_task:
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
        
        self.state = BeaconTrackerState.IDLE
        logger.info("Stopped tracking beacon")
    
    def _start_continuous_scanning(self):
        """Start continuous BLE scanning in the background."""
        if self.scanning_task and not self.scanning_task.done():
            # Already scanning
            return
        
        self.scanning_task = asyncio.create_task(self._continuous_scan_loop())
        logger.debug("Started continuous BLE scanning")
    
    async def _continuous_scan_loop(self):
        """Background task for continuous BLE scanning."""
        try:
            while self.running and self.state in [BeaconTrackerState.TRACKING, BeaconTrackerState.APPROACHING, BeaconTrackerState.CALIBRATING]:
                if self.paused:
                    await asyncio.sleep(0.5)
                    continue
                
                with self.scan_lock:
                    # Start a scan
                    await self.scanner.start()
                    await asyncio.sleep(self.scan_interval)
                    await self.scanner.stop()
                
                # Small pause before the next scan
                await asyncio.sleep(0.1)
        except asyncio.CancelledError:
            logger.debug("Continuous scan loop cancelled")
            raise
        except Exception as e:
            logger.error(f"Error in continuous scan loop: {e}")
            if self.scanner:
                await self.scanner.stop()
    
    async def approach_beacon(self):
        """
        Begin approaching the target beacon.
        Uses RSSI readings to navigate toward the beacon.
        """
        if not await self.start_tracking():
            return False
        
        self.state = BeaconTrackerState.APPROACHING
        logger.info(f"Beginning approach to beacon: {self.target_beacon_name}")
        
        # Loop until we reach the beacon or are stopped
        while self.running and self.state == BeaconTrackerState.APPROACHING:
            if self.paused:
                await asyncio.sleep(0.5)
                continue
            
            # Check if we have recent target beacon data
            if not self.target_beacon_data or time.time() - self.target_beacon_data.get('last_seen', 0) > 5:
                logger.warning("Target beacon not detected recently")
                self.motion_controller.stop()
                
                # Search for the beacon
                await self._search_for_beacon()
                continue
            
            # Calculate the average RSSI
            avg_rssi = statistics.mean(self.rssi_readings) if self.rssi_readings else None
            
            if avg_rssi is None:
                logger.warning("No RSSI readings available")
                self.motion_controller.stop()
                await asyncio.sleep(1)
                continue
            
            # Decide on movement based on RSSI
            await self._plan_movement_based_on_rssi(avg_rssi)
            
            # Brief pause before the next decision
            await asyncio.sleep(0.5)
        
        return True
    
    async def _plan_movement_based_on_rssi(self, current_rssi: float):
        """
        Improved movement planning based on RSSI values.
        Uses dynamic movement durations and improved decision making.
        
        Args:
            current_rssi: Current average RSSI value
        """
        # If signal is very strong, we're close
        if current_rssi > -45:  # Even closer threshold
            logger.info(f"Very close to beacon (RSSI: {current_rssi} dBm)")
            self.motion_controller.stop()
            await asyncio.sleep(1.0)  # Longer pause when very close
            return
        
        # If we have calibration data, use it
        if self.calibration_data['best_direction']:
            best_direction = self.calibration_data['best_direction']
            
            # Determine movement duration based on signal strength
            # Stronger signal = shorter movement time
            if current_rssi > -60:
                duration = 0.8  # Short movements when close
            elif current_rssi > -70:
                duration = 1.2  # Medium movements
            else:
                duration = 1.8  # Longer movements when far
            
            # Move in the best direction based on calibration
            await self.continuous_movement(best_direction, duration)
            
            logger.info(f"Moving {best_direction.lower()} for {duration}s, RSSI: {current_rssi} dBm")
        else:
            # No calibration data, use simpler approach
            # Adjust threshold based on signal strength
            adjusted_threshold = max(self.rssi_threshold, current_rssi - 10)
            
            if current_rssi < adjusted_threshold:
                await self._perform_signal_search()
            else:
                # Move forward with dynamic duration
                duration = 1.5 if current_rssi < -70 else 1.0
                self.motion_controller.forward(self.movement_speed)
                await asyncio.sleep(duration)
                self.motion_controller.stop()
                logger.info(f"Moving forward for {duration}s, RSSI: {current_rssi} dBm")
        
        # Wait for stable readings
        await asyncio.sleep(0.3)  # Reduced from 0.5 for faster response

    async def _perform_signal_search(self):
        """
        Optimized search pattern to find better signal.
        Tests all directions and moves in the best one.
        """
        direction_enum = self.motion_controller.current_direction.__class__
        directions = [
            (direction_enum.FORWARD, "forward"),
            (direction_enum.LEFT, "left"),
            (direction_enum.RIGHT, "right"),
            # Only include BACKWARD if really needed
            (direction_enum.BACKWARD, "backward")
        ]
        
        best_rssi = float('-inf')
        best_dir = None
        
        # Save initial readings to compare against
        initial_readings = self.rssi_readings[-min(len(self.rssi_readings), self.rssi_samples):]
        initial_avg = statistics.mean(initial_readings) if initial_readings else None
        
        for direction, name in directions:
            # Move in this direction
            if direction == direction_enum.FORWARD:
                self.motion_controller.forward(self.movement_speed)
            elif direction == direction_enum.BACKWARD:
                self.motion_controller.backward(self.movement_speed)
            elif direction == direction_enum.LEFT:
                self.motion_controller.turn_left(self.movement_speed)
            elif direction == direction_enum.RIGHT:
                self.motion_controller.turn_right(self.movement_speed)
            
            logger.debug(f"Testing direction: {name}")
            
            # Move for a short time - shorter for faster search
            await asyncio.sleep(0.8)  # Reduced from 1.0
            self.motion_controller.stop()
            
            # Wait for stable readings - shorter for faster search
            await asyncio.sleep(0.3)  # Reduced from 0.5
            
            # Take RSSI readings
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.rssi_samples):]
            if readings:
                avg_rssi = statistics.mean(readings)
                logger.debug(f"Direction {name}: avg RSSI = {avg_rssi} dBm")
                
                # Track readings by direction
                self.rssi_by_direction[direction.name].append(avg_rssi)
                
                # Find best direction with significant improvement
                if avg_rssi > best_rssi:
                    # Require significant improvement for backward direction
                    if direction == direction_enum.BACKWARD and initial_avg is not None:
                        # Only choose backward if it's significantly better
                        if avg_rssi > initial_avg + 5:
                            best_rssi = avg_rssi
                            best_dir = direction
                    else:
                        best_rssi = avg_rssi
                        best_dir = direction
        
        # Move in the best direction found
        if best_dir:
            logger.info(f"Best direction: {best_dir.name.lower()} with RSSI: {best_rssi} dBm")
            
            # Move longer in the best direction
            if best_dir == direction_enum.FORWARD:
                self.motion_controller.forward(self.movement_speed)
            elif best_dir == direction_enum.BACKWARD:
                self.motion_controller.backward(self.movement_speed)
            elif best_dir == direction_enum.LEFT:
                self.motion_controller.turn_left(self.movement_speed)
            elif best_dir == direction_enum.RIGHT:
                self.motion_controller.turn_right(self.movement_speed)
            
            # Move longer based on improvement amount
            improvement = best_rssi - (initial_avg or -100)
            duration = min(2.0, max(1.0, 1.0 + improvement/10))
            
            await asyncio.sleep(duration)
            self.motion_controller.stop()
        else:
            logger.warning("No good direction found")
            # Try a spiral search pattern
            await self._spiral_search()  

    async def _spiral_search(self, max_steps: int = 5):
        """
        Perform a spiral search pattern to find the beacon.
        
        Args:
            max_steps: Maximum number of steps in the spiral
        """
        logger.info("Performing spiral search for beacon")
        direction_enum = self.motion_controller.current_direction.__class__
        
        # Spiral pattern: Move forward, turn, move forward longer, turn...
        step_size = 1.0  # Base step time in seconds
        
        for i in range(1, max_steps + 1):
            # Move forward
            self.motion_controller.forward(self.movement_speed)
            await asyncio.sleep(step_size * i)
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
            
            # Check if we found the beacon with good signal
            if self.target_beacon_data and time.time() - self.target_beacon_data.get('last_seen', 0) < 1:
                if self.current_rssi and self.current_rssi > self.rssi_threshold:
                    logger.info(f"Found beacon during spiral search with RSSI: {self.current_rssi}")
                    return
            
            # Turn right
            self.motion_controller.turn_right(self.movement_speed)
            await asyncio.sleep(0.5)  # 90 degree turn
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
            
            # Move forward again
            self.motion_controller.forward(self.movement_speed)
            await asyncio.sleep(step_size * i)
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
            
            # Check if we found the beacon
            if self.target_beacon_data and time.time() - self.target_beacon_data.get('last_seen', 0) < 1:
                if self.current_rssi and self.current_rssi > self.rssi_threshold:
                    logger.info(f"Found beacon during spiral search with RSSI: {self.current_rssi}")
                    return
            
            # Turn right
            self.motion_controller.turn_right(self.movement_speed)
            await asyncio.sleep(0.5)  # 90 degree turn
            self.motion_controller.stop()
            await asyncio.sleep(0.5)  # Pause to scan
        
        logger.warning("Spiral search completed without finding the beacon")
    
    async def _search_for_beacon(self):
        """Search for the beacon if it's lost."""
        logger.info("Searching for lost beacon")
        
        # First, do a 360-degree scan
        for _ in range(4):  # 4 x 90 degrees = 360 degrees
            self.motion_controller.turn_right(self.movement_speed)
            await asyncio.sleep(1.0)
            self.motion_controller.stop()
            await asyncio.sleep(1.0)  # Pause to scan
            
            # Check if we found the beacon
            if self.target_beacon_data and time.time() - self.target_beacon_data.get('last_seen', 0) < 2:
                logger.info(f"Found beacon during rotation search with RSSI: {self.current_rssi}")
                return
        
        # If not found, try a spiral search
        await self._spiral_search()
    
    async def calibrate(self, callback: Callable = None):
        """
        Perform a calibration routine to optimize beacon tracking.
        This includes measuring RSSI at different rotations and distances.
        
        Args:
            callback: Optional callback function for progress updates
        
        Returns:
            Calibration results dictionary
        """
        if not self.target_beacon_name and not self.target_beacon_pattern:
            logger.error("No target beacon specified. Use set_target_beacon() or set_target_beacon_pattern() first.")
            return None
        
        prev_state = self.state
        self.state = BeaconTrackerState.CALIBRATING
        self.calibration_phase = CalibrationPhase.INIT
        
        logger.info(f"Starting calibration for beacon: {self.target_beacon_name}")
        
        # Start continuous scanning
        self._start_continuous_scanning()
        
        # Initialize calibration data
        self.calibration_data = {
            'rotation_map': {},
            'distance_map': {},
            'best_direction': None,
            'signal_peak': None,
        }
        
        # Reset RSSI readings
        self.rssi_readings = []
        
        try:
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
            
            logger.info("Calibration completed successfully")
            logger.info(f"Best direction: {self.calibration_data['best_direction']}")
            
            return self.calibration_data
            
        except Exception as e:
            logger.error(f"Error during calibration: {e}")
            if callback:
                callback(f"Calibration error: {str(e)}")
            return None
        
        finally:
            # Restore previous state
            self.state = prev_state
            
            # Stop continuous scanning if we weren't tracking before
            if prev_state not in [BeaconTrackerState.TRACKING, BeaconTrackerState.APPROACHING]:
                if self.scanning_task:
                    self.scanning_task.cancel()
                    try:
                        await self.scanning_task
                    except asyncio.CancelledError:
                        pass
                    self.scanning_task = None
                
                if self.scanner:
                    await self.scanner.stop()
    
    async def _calibrate_rotation(self, steps: int = 8):
        """
        Perform rotational calibration.
        Measures RSSI while rotating in place.
        
        Args:
            steps: Number of rotation steps (default divides 360° into 8 parts)
        """
        logger.info("Starting rotational calibration")
        
        rotation_map = {}
        degrees_per_step = 360 / steps
        
        # Make sure we start from a known position
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        for i in range(steps):
            current_degrees = i * degrees_per_step
            logger.debug(f"Rotating to {current_degrees} degrees")
            
            # Rotate to the desired position
            self.motion_controller.turn_right(self.movement_speed)
            await asyncio.sleep(degrees_per_step / 90)  # Assuming 90 degrees takes about 1 second
            self.motion_controller.stop()
            
            # Wait for stable readings
            await asyncio.sleep(1.0)
            
            # Take RSSI readings at this position
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.rssi_samples):]
            if readings:
                avg_rssi = statistics.mean(readings)
                rotation_map[current_degrees] = avg_rssi
                logger.debug(f"Position {current_degrees}°: RSSI = {avg_rssi} dBm")
        
        # Store rotation data
        self.calibration_data['rotation_map'] = rotation_map
        
        # Return to the position with the strongest signal
        if rotation_map:
            best_degrees = max(rotation_map.items(), key=lambda x: x[1])[0]
            logger.info(f"Best signal at {best_degrees} degrees, RSSI: {rotation_map[best_degrees]} dBm")
            
            # Calculate how much to rotate to get back to the best position
            current_degrees = (steps - 1) * degrees_per_step  # We're at the last position
            degrees_to_turn = (best_degrees - current_degrees) % 360
            
            if degrees_to_turn > 180:
                # Shorter to turn left
                self.motion_controller.turn_left(self.movement_speed)
                await asyncio.sleep((360 - degrees_to_turn) / 90)
            else:
                # Shorter to turn right
                self.motion_controller.turn_right(self.movement_speed)
                await asyncio.sleep(degrees_to_turn / 90)
            
            self.motion_controller.stop()
            await asyncio.sleep(1.0)
    
    async def _calibrate_distance(self, steps: int = 3, step_time: float = 1.0):
        """
        Perform distance calibration.
        Measures RSSI at different distances from the current position.
        
        Args:
            steps: Number of steps to move
            step_time: Time to move for each step (in seconds)
        """
        logger.info("Starting distance calibration")
        
        distance_map = {}
        
        # Make sure we start from a stopped position
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        # First, get current position reading
        readings = self.rssi_readings[-min(len(self.rssi_readings), self.rssi_samples):]
        if readings:
            avg_rssi = statistics.mean(readings)
            distance_map[0] = avg_rssi
            logger.debug(f"Base position: RSSI = {avg_rssi} dBm")
        
        # Test moving forward
        for i in range(1, steps + 1):
            # Move forward
            self.motion_controller.forward(self.movement_speed)
            await asyncio.sleep(step_time)
            self.motion_controller.stop()
            
            # Wait for stable readings
            await asyncio.sleep(1.0)
            
            # Take RSSI readings at this position
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.rssi_samples):]
            if readings:
                avg_rssi = statistics.mean(readings)
                distance_map[i] = avg_rssi
                logger.debug(f"Forward step {i}: RSSI = {avg_rssi} dBm")
        
        # Return to the base position
        self.motion_controller.backward(self.movement_speed)
        await asyncio.sleep(step_time * steps)
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        # Test moving backward
        for i in range(1, steps + 1):
            # Move backward
            self.motion_controller.backward(self.movement_speed)
            await asyncio.sleep(step_time)
            self.motion_controller.stop()
            
            # Wait for stable readings
            await asyncio.sleep(1.0)
            
            # Take RSSI readings at this position
            readings = self.rssi_readings[-min(len(self.rssi_readings), self.rssi_samples):]
            if readings:
                avg_rssi = statistics.mean(readings)
                distance_map[-i] = avg_rssi
                logger.debug(f"Backward step {i}: RSSI = {avg_rssi} dBm")
        
        # Return to the base position
        self.motion_controller.forward(self.movement_speed)
        await asyncio.sleep(step_time * steps)
        self.motion_controller.stop()
        await asyncio.sleep(1.0)
        
        # Store distance data
        self.calibration_data['distance_map'] = distance_map
    
    async def _analyze_calibration(self):
        """Analyze the calibration data to determine optimal movement strategies."""
        logger.info("Analyzing calibration data")
        direction_enum = self.motion_controller.current_direction.__class__
        
        # Analyze rotation data
        rotation_map = self.calibration_data['rotation_map']
        if rotation_map:
            # Find the direction with the strongest signal
            best_degrees, best_rssi = max(rotation_map.items(), key=lambda x: x[1])
            logger.info(f"Best rotation: {best_degrees} degrees with RSSI: {best_rssi} dBm")
            
            # Store the peak signal strength
            self.calibration_data['signal_peak'] = best_rssi
        
        # Analyze distance data
        distance_map = self.calibration_data['distance_map']
        if distance_map:
            # Find the best step direction
            best_step, best_rssi = max(distance_map.items(), key=lambda x: x[1])
            
            if best_step > 0:
                best_direction = direction_enum.FORWARD.name
            elif best_step < 0:
                best_direction = direction_enum.BACKWARD.name
            else:
                # If 0 is best, determine from rotation
                if rotation_map:
                    best_degrees = max(rotation_map.items(), key=lambda x: x[1])[0]
                    if 45 <= best_degrees <= 135:
                        best_direction = direction_enum.RIGHT.name
                    elif 225 <= best_degrees <= 315:
                        best_direction = direction_enum.LEFT.name
                    else:
                        best_direction = direction_enum.FORWARD.name
                else:
                    best_direction = direction_enum.FORWARD.name
            
            logger.info(f"Best movement direction: {best_direction}")
            self.calibration_data['best_direction'] = best_direction
    
    def get_state(self):
        """
        Get the current state of the beacon tracker.
        
        Returns:
            Dictionary with current state
        """
        target_info = self.target_beacon_name
        if not target_info and self.target_beacon_pattern:
            target_info = f"pattern:{self.target_beacon_pattern}"
            
        # Get names of detected beacons for reporting
        beacon_names = list(self.detected_beacons.keys())
        
        state_data = {
            'state': self.state.value,
            'target_beacon': target_info,
            'target_beacon_data': self.target_beacon_data.get('display_name') if self.target_beacon_data else None,
            'rssi': self.current_rssi,
            'avg_rssi': statistics.mean(self.rssi_readings) if self.rssi_readings else None,
            'detected_beacons_count': len(self.detected_beacons),
            'detected_beacons': beacon_names[:10],  # Limit to first 10 for display
            'is_calibrated': self.calibration_phase == CalibrationPhase.COMPLETE,
            'best_direction': self.calibration_data.get('best_direction'),
            'timestamp': time.time()
        }
        
        return state_data

    async def continuous_movement(self, direction_name, duration=1.5):
        """
        Move continuously in the specified direction for better momentum.
        
        Args:
            direction_name: Direction to move in (FORWARD, BACKWARD, etc.)
            duration: Time to move in seconds
        """
        direction_enum = self.motion_controller.current_direction.__class__
        
        # Get the direction enum value from the name
        direction = getattr(direction_enum, direction_name)
        
        # Execute the move with the specific direction
        if direction == direction_enum.FORWARD:
            self.motion_controller.forward(self.movement_speed)
        elif direction == direction_enum.BACKWARD:
            self.motion_controller.backward(self.movement_speed)
        elif direction == direction_enum.LEFT:
            self.motion_controller.turn_left(self.movement_speed)
        elif direction == direction_enum.RIGHT:
            self.motion_controller.turn_right(self.movement_speed)
        
        # Move for the specified duration
        await asyncio.sleep(duration)
        
        # Stop
        self.motion_controller.stop()