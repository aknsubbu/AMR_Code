"""
Ultrasonic Manager for processing ultrasonic sensor data.
"""
import logging
import time
import threading
import numpy as np
from collections import deque

logger = logging.getLogger(__name__)

class UltrasonicManager:
    """
    Manages and processes ultrasonic sensor readings.
    Provides filtered distance measurements and obstacle detection.
    """
    
    def __init__(self, serial_communicator, event_bus, history_size=5,use_rpi_sensors=True):
        """
        Initialize the ultrasonic manager.
        
        Args:
            serial_communicator: SerialCommunicator instance
            event_bus: EventBus for inter-module communication
            history_size: Number of readings to keep in history
        """
        self.serial_communicator = serial_communicator
        self.event_bus = event_bus
        self.history_size = history_size
        self.sensor_count = 3  # Front1, Front2, (Placeholder for compatibility)
        
        # Initialize history for each sensor
        self.readings_history = [deque(maxlen=history_size) for _ in range(self.sensor_count)]
        self.last_readings = [None] * self.sensor_count
        self.last_update = 0
        self.running = False
        self.update_thread = None
        self.lock = threading.RLock()

        self.use_rpi_sensors = use_rpi_sensors
        if use_rpi_sensors:
            from modules.sensor_fusion.rpi_ultrasonic import RPiUltrasonicSensor
            self.sensor1 = RPiUltrasonicSensor(17, 27, "front1")
            self.sensor2 = RPiUltrasonicSensor(22, 23, "front2")
            logger.info("Using RPi GPIO for ultrasonic sensors")
        
        # Safe distances (in cm)
        self.safe_distances = [15, 15, 15]  # Safe distances for each sensor
        
        # Register event handlers
        self.event_bus.register('ultrasonic_readings', self.handle_ultrasonic_readings)
        self.event_bus.register('request_distances', self.handle_request_distances)
        
        logger.info("Ultrasonic manager initialized with dual front sensors")
        
        # Start background thread for periodic updates
        self.start_updates()
    
    def start_updates(self, interval=0.5):
        """
        Start periodic sensor updates.
        
        Args:
            interval: Update interval in seconds
        """
        if self.running:
            logger.warning("Updates already running")
            return
        
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, args=(interval,), daemon=True)
        self.update_thread.start()
        logger.debug(f"Started ultrasonic update thread with interval {interval}s")
    
    def stop_updates(self):
        """Stop periodic sensor updates."""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=2.0)
        logger.debug("Stopped ultrasonic update thread")
    
    def _update_loop(self, interval):
        """
        Background thread for periodic sensor updates.
        
        Args:
            interval: Update interval in seconds
        """
        while self.running:
            self.request_readings()
            time.sleep(interval)
    
    def request_readings(self):
        """Request sensor readings"""
        if self.use_rpi_sensors:
            # Read directly from GPIO
            front1 = self.sensor1.get_distance()
            front2 = self.sensor2.get_distance()
            readings = [front1, front2, 0]  # Third value placeholder
            self.handle_ultrasonic_readings(readings)
            return True
        else:
            # Use Arduino via serial
            return self.serial_communicator.send_command("D")
    
    def handle_ultrasonic_readings(self, readings):
        """
        Handle new ultrasonic readings.
        
        Args:
            readings: List of distance readings [front1, front2, placeholder]
        """
        if not readings or len(readings) < 2:  # Need at least the two front sensors
            logger.warning(f"Incomplete readings: {readings}")
            return
        
        # Ensure we have 3 readings for compatibility (padding if needed)
        padded_readings = list(readings)
        while len(padded_readings) < self.sensor_count:
            padded_readings.append(-1)
        
        with self.lock:
            # Update histories
            for i in range(self.sensor_count):
                if padded_readings[i] > 0:  # Ignore invalid readings
                    self.readings_history[i].append(padded_readings[i])
            
            # Update last readings
            self.last_readings = padded_readings.copy()
            self.last_update = time.time()
        
        # Check for obstacles
        self._check_obstacles()
    
    def _check_obstacles(self):
        """
        Check for obstacles based on current readings.
        Publishes obstacle detected events if obstacles found.
        """
        with self.lock:
            if not self.last_readings:
                return
            
            # Special handling for dual front sensors (take minimum distance)
            front_readings = [r for r in [self.last_readings[0], self.last_readings[1]] if r > 0]
            if front_readings:
                min_front_distance = min(front_readings)
                if min_front_distance < self.safe_distances[0]:
                    # Front obstacle detected
                    # Calculate confidence (average confidence from both sensors)
                    confidence = (self._calculate_confidence(0) + self._calculate_confidence(1)) / 2
                    
                    if confidence > 0.5:  # Only report if confidence is high enough
                        logger.info(f"Obstacle detected by front sensors at {min_front_distance}cm (confidence: {confidence:.2f})")
                        
                        # Publish obstacle detected event
                        self.event_bus.publish('obstacle_detected', {
                            'sensor': 'front',
                            'distance': min_front_distance,
                            'confidence': confidence,
                            'timestamp': time.time(),
                            'message': f"Front obstacle at {min_front_distance}cm"
                        })
            
            # Check third sensor (placeholder for compatibility)
            if self.sensor_count > 2 and self.last_readings[2] > 0 and self.last_readings[2] < self.safe_distances[2]:
                # Process placeholder sensor (not used in current hardware)
                pass
    
    def _calculate_confidence(self, sensor_idx):
        """
        Calculate confidence level for a sensor reading.
        
        Args:
            sensor_idx: Sensor index
            
        Returns:
            Confidence value between 0.0 and 1.0
        """
        history = list(self.readings_history[sensor_idx])
        if not history:
            return 0.0
        
        # Calculate confidence based on:
        # 1. Number of consistent readings
        # 2. Standard deviation of readings
        # 3. Recency of readings
        
        # More readings = higher confidence
        count_factor = min(1.0, len(history) / self.history_size)
        
        # Lower standard deviation = higher confidence
        if len(history) > 1:
            std_dev = np.std(history)
            max_expected_std = 5.0  # cm
            std_factor = max(0.0, 1.0 - (std_dev / max_expected_std))
        else:
            std_factor = 0.5  # Default if only one reading
        
        # Combine factors (can be weighted differently)
        return (count_factor * 0.4) + (std_factor * 0.6)
    
    def get_current_distances(self):
        """
        Get the current filtered distances.
        
        Returns:
            List of [front, front2, placeholder] distances in cm
        """
        with self.lock:
            filtered = []
            
            for i in range(self.sensor_count):
                history = list(self.readings_history[i])
                
                if not history:
                    filtered.append(-1)  # No data
                elif len(history) == 1:
                    filtered.append(history[0])  # Only one reading
                else:
                    # Use median filter for robustness against outliers
                    filtered.append(np.median(history))
            
            return filtered
    
    def get_distance(self, sensor):
        """
        Get the current filtered distance for a specific sensor.
        
        Args:
            sensor: Sensor name ('front', 'front2') or index
            
        Returns:
            Distance in cm or -1 if no data
        """
        # Special handling for 'front' to return minimum of both front sensors
        if isinstance(sensor, str) and sensor.lower() == 'front':
            with self.lock:
                front1_history = list(self.readings_history[0])
                front2_history = list(self.readings_history[1])
                
                front1_distance = -1
                front2_distance = -1
                
                # Get median distance from each front sensor
                if front1_history:
                    front1_distance = np.median(front1_history) if len(front1_history) > 1 else front1_history[0]
                
                if front2_history:
                    front2_distance = np.median(front2_history) if len(front2_history) > 1 else front2_history[0]
                
                # Return minimum valid distance
                valid_distances = [d for d in [front1_distance, front2_distance] if d > 0]
                return min(valid_distances) if valid_distances else -1
        
        # Convert name to index if needed
        if isinstance(sensor, str):
            sensor_map = {'front1': 0, 'front2': 1}
            if sensor.lower() not in sensor_map:
                logger.error(f"Invalid sensor name: {sensor}")
                return -1
            sensor_idx = sensor_map[sensor.lower()]
        else:
            sensor_idx = sensor
        
        # Check index
        if sensor_idx < 0 or sensor_idx >= self.sensor_count:
            logger.error(f"Invalid sensor index: {sensor_idx}")
            return -1
        
        with self.lock:
            history = list(self.readings_history[sensor_idx])
            
            if not history:
                return -1  # No data
            elif len(history) == 1:
                return history[0]  # Only one reading
            else:
                # Use median filter for robustness
                return np.median(history)
            
    def is_obstacle_detected(self, sensor=None):
        """
        Check if an obstacle is detected.
        
        Args:
            sensor: Specific sensor to check (None for any sensor)
            
        Returns:
            True if obstacle detected, False otherwise
        """
        if sensor is None:
            # Check all sensors
            # Special handling for front sensors
            front_distance = self.get_distance('front') # This will check both front sensors
            if front_distance > 0 and front_distance < self.safe_distances[0]:
                return True
            
            # Check third sensor (placeholder, if present)
            if self.sensor_count > 2:
                distances = self.get_current_distances()
                if distances[2] > 0 and distances[2] < self.safe_distances[2]:
                    return True
            
            return False
        elif sensor == 'front':
            # Check front sensors (minimum of both)
            distance = self.get_distance('front')
            return distance > 0 and distance < self.safe_distances[0]
        else:
            # Check specific sensor
            if isinstance(sensor, str):
                sensor_map = {'front1': 0, 'front2': 1}
                if sensor.lower() not in sensor_map:
                    logger.error(f"Invalid sensor name: {sensor}")
                    return False
                sensor_idx = sensor_map[sensor.lower()]
            else:
                sensor_idx = sensor
            
            if sensor_idx < 0 or sensor_idx >= self.sensor_count:
                return False
                
            distance = self.get_distance(sensor_idx)
            return distance > 0 and distance < self.safe_distances[sensor_idx]
    
    def handle_request_distances(self, data=None):
        """
        Handle request for distance measurements.
        
        Args:
            data: Optional request parameters
        """
        # Request new readings from Arduino
        self.request_readings()
        
        # Get current distances
        distances = self.get_current_distances()
        
        # Get combined front distance (minimum of both front sensors)
        front_distance = self.get_distance('front')
        
        # Publish distance data
        self.event_bus.publish('distance_data', {
            'front': front_distance,
            'front1': distances[0],
            'front2': distances[1],
            'timestamp': time.time()
        })