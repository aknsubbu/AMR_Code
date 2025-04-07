"""
IR Sensor Manager for processing IR sensor data.
"""
import logging
import time
import threading
from collections import deque

logger = logging.getLogger(__name__)

class IRSensorManager:
    """
    Manages and processes IR sensor readings.
    Handles both cliff detection and rear obstacle detection.
    """
    
    def __init__(self, serial_communicator, event_bus, history_size=5):
        """
        Initialize the IR sensor manager.
        
        Args:
            serial_communicator: SerialCommunicator instance
            event_bus: EventBus for inter-module communication
            history_size: Number of readings to keep in history
        """
        self.serial_communicator = serial_communicator
        self.event_bus = event_bus
        self.history_size = history_size
        
        # IR sensors (back and cliff)
        self.back_history = deque(maxlen=history_size)
        self.cliff_history = deque(maxlen=history_size)
        
        self.last_back = None
        self.last_cliff = None
        self.last_update = 0
        
        self.lock = threading.RLock()
        
        # Register event handlers
        self.event_bus.register('ir_readings', self.handle_ir_readings)
        
        logger.info("IR sensor manager initialized")
    
    def request_readings(self):
        """
        Request sensor readings from the Arduino.
        
        Returns:
            True if request sent, False otherwise
        """
        # Send data request command to Arduino
        return self.serial_communicator.send_command("D")
    
    def handle_ir_readings(self, readings):
        """
        Handle new IR sensor readings.
        
        Args:
            readings: Dict with 'back' and 'cliff' values
        """
        if not readings or 'back' not in readings or 'cliff' not in readings:
            logger.warning("Invalid IR readings")
            return
        
        with self.lock:
            # Update histories
            self.back_history.append(readings['back'])
            self.cliff_history.append(readings['cliff'])
            
            # Update last values
            self.last_back = readings['back']
            self.last_cliff = readings['cliff']
            self.last_update = time.time()
        
        # Check for obstacles
        self._check_obstacles()
    
    def _check_obstacles(self):
        """
        Check for obstacles and cliffs based on current readings.
        Publishes events if obstacles or cliffs are detected.
        """
        with self.lock:
            if self.last_back is None or self.last_cliff is None:
                return
            
            # Check for back obstacle (HIGH = obstacle detected)
            if self.last_back == 1 and self._is_reading_stable('back'):
                logger.info("Back obstacle detected")
                
                # Publish obstacle detected event
                self.event_bus.publish('obstacle_detected', {
                    'sensor': 'ir_back',
                    'detected': True,
                    'confidence': self._calculate_confidence('back'),
                    'timestamp': time.time(),
                    'message': "Obstacle detected behind robot"
                })
            
            # Check for cliff (LOW = cliff detected)
            if self.last_cliff == 0 and self._is_reading_stable('cliff'):
                logger.info("Cliff detected")
                
                # Publish cliff detected event
                self.event_bus.publish('cliff_detected', {
                    'sensor': 'ir_cliff',
                    'detected': True,
                    'confidence': self._calculate_confidence('cliff'),
                    'timestamp': time.time(),
                    'message': "Cliff detected"
                })
                
                # Also publish as obstacle (for general obstacle avoidance)
                self.event_bus.publish('obstacle_detected', {
                    'sensor': 'ir_cliff',
                    'detected': True,
                    'confidence': self._calculate_confidence('cliff'),
                    'timestamp': time.time(),
                    'message': "Cliff detected - Emergency stop"
                })
    
    def _is_reading_stable(self, sensor):
        """
        Check if readings are stable (same value for multiple readings).
        
        Args:
            sensor: Sensor name ('back' or 'cliff')
            
        Returns:
            True if readings are stable, False otherwise
        """
        history = self.back_history if sensor == 'back' else self.cliff_history
        
        if len(history) < 2:
            return False
        
        # Check last few readings
        last_readings = list(history)[-3:]
        return all(r == last_readings[0] for r in last_readings)
    
    def _calculate_confidence(self, sensor):
        """
        Calculate confidence level for a sensor reading.
        
        Args:
            sensor: Sensor name ('back' or 'cliff')
            
        Returns:
            Confidence value between 0.0 and 1.0
        """
        history = self.back_history if sensor == 'back' else self.cliff_history
        
        if not history:
            return 0.0
        
        # For digital sensors, confidence is based on consistency
        readings = list(history)
        count = sum(1 for r in readings if r == readings[-1])
        confidence = count / len(readings)
        
        return confidence
    
    def is_back_obstacle_detected(self):
        """
        Check if a back obstacle is detected.
        
        Returns:
            True if obstacle detected, False otherwise
        """
        with self.lock:
            if not self.back_history:
                return False
            
            # Get most common reading from recent history
            readings = list(self.back_history)
            back_status = max(set(readings), key=readings.count)
            
            return back_status == 1  # HIGH = obstacle
    
    def is_cliff_detected(self):
        """
        Check if a cliff is detected.
        
        Returns:
            True if cliff detected, False otherwise
        """
        with self.lock:
            if not self.cliff_history:
                return False
            
            # Get most common reading from recent history
            readings = list(self.cliff_history)
            cliff_status = max(set(readings), key=readings.count)
            
            return cliff_status == 0  # LOW = cliff
    
    def get_sensor_status(self):
        """
        Get the current status of all IR sensors.
        
        Returns:
            Dict with sensor status
        """
        with self.lock:
            return {
                'back_obstacle': self.is_back_obstacle_detected(),
                'cliff': self.is_cliff_detected(),
                'last_update': self.last_update,
                'back_confidence': self._calculate_confidence('back'),
                'cliff_confidence': self._calculate_confidence('cliff')
            }