"""
Position Estimator for determining the robot's position using BLE beacons.
Now updated to use device names as identifiers instead of UUIDs.
"""
import logging
import math
import numpy as np
from datetime import datetime
from threading import Lock

logger = logging.getLogger(__name__)

class PositionEstimator:
    """
    Estimates the robot's position using BLE beacon signal strengths.
    Implements several positioning algorithms:
    1. Nearest beacon
    2. Weighted centroid
    3. Trilateration
    """
    
    def __init__(self, beacon_registry, beacon_scanner, event_bus, method='weighted'):
        """
        Initialize the position estimator.
        
        Args:
            beacon_registry: BeaconRegistry instance
            beacon_scanner: BeaconScanner instance
            event_bus: EventBus for inter-module communication
            method: Positioning method ('nearest', 'weighted', 'trilateration')
        """
        self.beacon_registry = beacon_registry
        self.beacon_scanner = beacon_scanner
        self.event_bus = event_bus
        self.method = method
        self.current_position = {'x': 0, 'y': 0}
        self.current_orientation = 0  # Degrees from north
        self.position_history = []
        self.last_update = datetime.now()
        self.update_lock = Lock()
        
        # Register event handlers
        self.event_bus.register('beacon_scan_complete', self.handle_beacon_scan)
        self.event_bus.register('request_position', self.handle_position_request)
        
        logger.info(f"Position estimator initialized using {method} method")
    
    def update_position(self):
        """
        Update the current position using the latest beacon data.
        This can be called explicitly or will be triggered by scan events.
        """
        visible_beacons = self.beacon_scanner.get_visible_beacons()
        if not visible_beacons:
            logger.warning("No beacons visible for position update")
            return False
        
        with self.update_lock:
            if self.method == 'nearest':
                success = self._update_position_nearest(visible_beacons)
            elif self.method == 'weighted':
                success = self._update_position_weighted(visible_beacons)
            elif self.method == 'trilateration':
                success = self._update_position_trilateration(visible_beacons)
            else:
                logger.error(f"Unknown positioning method: {self.method}")
                success = False
            
            if success:
                self.last_update = datetime.now()
                # Keep a history of positions (last 20)
                self.position_history.append(self.current_position.copy())
                if len(self.position_history) > 20:
                    self.position_history.pop(0)
                
                # Optional: Attempt to derive orientation from position history
                if len(self.position_history) >= 2:
                    self._update_orientation()
                
                # Publish position update event
                self.event_bus.publish('position_updated', self.get_current_position())
                return True
        
        return False
    
    def _update_position_nearest(self, visible_beacons):
        """
        Update position based on the nearest beacon (strongest RSSI).
        
        Args:
            visible_beacons: Dict of {name: {'rssi': value, 'mac_address': addr}}
            
        Returns:
            True if position was updated, False otherwise
        """
        if not visible_beacons:
            return False
        
        # Find the strongest beacon
        strongest_name = max(visible_beacons, key=lambda name: visible_beacons[name]['rssi'])
        
        # Look up its position
        beacon = self.beacon_registry.get_beacon(strongest_name)
        if not beacon:
            # Try finding by MAC address as fallback
            mac_address = visible_beacons[strongest_name]['mac_address']
            beacon = self.beacon_registry.get_beacon_by_mac(mac_address)
            if not beacon:
                logger.warning(f"Unknown beacon: {strongest_name} (MAC: {mac_address})")
                return False
        
        # Update position to that beacon's position
        self.current_position = {'x': beacon['x'], 'y': beacon['y']}
        logger.debug(f"Position updated to {self.current_position} (nearest beacon method)")
        return True
    
    def _update_position_weighted(self, visible_beacons):
        """
        Update position based on weighted centroid of visible beacons.
        
        Args:
            visible_beacons: Dict of {name: {'rssi': value, 'mac_address': addr}}
            
        Returns:
            True if position was updated, False otherwise
        """
        if not visible_beacons:
            return False
        
        # Convert RSSI to weights and get beacon positions
        weights = []
        positions = []
        
        for name, data in visible_beacons.items():
            beacon = self.beacon_registry.get_beacon(name)
            if not beacon:
                # Try finding by MAC as fallback
                beacon = self.beacon_registry.get_beacon_by_mac(data['mac_address'])
                if not beacon:
                    logger.debug(f"Skipping unknown beacon: {name} (MAC: {data['mac_address']})")
                    continue
            
            # Convert RSSI to weight (higher RSSI = higher weight)
            # RSSI is negative, so negate it and add offset to make values positive
            weight = abs(data['rssi']) - 100  # Simple linear weight
            
            weights.append(weight)
            positions.append((beacon['x'], beacon['y']))
        
        if not positions:
            logger.warning("No valid beacons found for weighted position")
            return False
        
        # Calculate weighted centroid
        weights_sum = sum(weights)
        if weights_sum == 0:
            logger.warning("Sum of weights is zero")
            return False
        
        x_weighted = sum(w * p[0] for w, p in zip(weights, positions)) / weights_sum
        y_weighted = sum(w * p[1] for w, p in zip(weights, positions)) / weights_sum
        
        # Update position
        self.current_position = {'x': round(x_weighted, 2), 'y': round(y_weighted, 2)}
        logger.debug(f"Position updated to {self.current_position} (weighted method)")
        return True
    
    def _update_position_trilateration(self, visible_beacons):
        """
        Update position using trilateration (distance-based positioning).
        Requires at least 3 beacons for accurate results.
        
        Args:
            visible_beacons: Dict of {name: {'rssi': value, 'mac_address': addr}}
            
        Returns:
            True if position was updated, False otherwise
        """
        if len(visible_beacons) < 3:
            logger.debug(f"Not enough beacons for trilateration: {len(visible_beacons)}")
            # Fall back to weighted method if not enough beacons
            return self._update_position_weighted(visible_beacons)
        
        # Convert RSSI to distances and get beacon positions
        beacons_with_distance = []
        
        for name, data in visible_beacons.items():
            beacon = self.beacon_registry.get_beacon(name)
            if not beacon:
                # Try finding by MAC as fallback
                beacon = self.beacon_registry.get_beacon_by_mac(data['mac_address'])
                if not beacon:
                    continue
            
            # Convert RSSI to distance using a simple path loss model
            # d = 10^((RSSI_1m - RSSI)/(10 * n))
            # where n is the path loss exponent, typically 2-4
            rssi = data['rssi']
            rssi_1m = -50  # RSSI at 1 meter (calibrate for your beacons)
            path_loss = 2.5  # Path loss exponent (adjust for your environment)
            
            distance = 10 ** ((rssi_1m - rssi) / (10 * path_loss))
            
            beacons_with_distance.append({
                'x': beacon['x'],
                'y': beacon['y'],
                'distance': distance
            })
        
        if len(beacons_with_distance) < 3:
            logger.debug(f"Not enough valid beacons for trilateration: {len(beacons_with_distance)}")
            return self._update_position_weighted(visible_beacons)
        
        try:
            # Solve the trilateration problem
            pos = self._trilaterate(beacons_with_distance)
            
            if pos:
                self.current_position = {'x': round(pos[0], 2), 'y': round(pos[1], 2)}
                logger.debug(f"Position updated to {self.current_position} (trilateration method)")
                return True
            else:
                logger.warning("Trilateration failed, falling back to weighted method")
                return self._update_position_weighted(visible_beacons)
                
        except Exception as e:
            logger.error(f"Trilateration error: {e}")
            return self._update_position_weighted(visible_beacons)
    
    def _trilaterate(self, beacons_with_distance):
        """
        Perform trilateration calculation using the first 3 beacons.
        
        Args:
            beacons_with_distance: List of beacons with distance estimates
            
        Returns:
            (x, y) tuple or None if calculation fails
        """
        # Sort beacons by distance for more stable results
        beacons_with_distance.sort(key=lambda b: b['distance'])
        
        # Use the first 3 beacons
        p1 = np.array([beacons_with_distance[0]['x'], beacons_with_distance[0]['y']])
        p2 = np.array([beacons_with_distance[1]['x'], beacons_with_distance[1]['y']])
        p3 = np.array([beacons_with_distance[2]['x'], beacons_with_distance[2]['y']])
        
        r1 = beacons_with_distance[0]['distance']
        r2 = beacons_with_distance[1]['distance']
        r3 = beacons_with_distance[2]['distance']
        
        # Create vectors from p1 to p2 and p3
        p1_to_p2 = p2 - p1
        p1_to_p3 = p3 - p1
        
        # Calculate dot products
        e_x = p1_to_p2 / np.linalg.norm(p1_to_p2)
        i = np.dot(e_x, p1_to_p3)
        e_y = (p1_to_p3 - i * e_x) / np.linalg.norm(p1_to_p3 - i * e_x)
        
        # Calculate the trilateration point
        d = np.linalg.norm(p1_to_p2)
        j = np.dot(e_y, p1_to_p3)
        
        # Calculate the coordinates
        x = (r1*r1 - r2*r2 + d*d) / (2*d)
        y = (r1*r1 - r3*r3 + i*i + j*j) / (2*j) - (i/j) * x
        
        # Convert back to original coordinate system
        result = p1 + x * e_x + y * e_y
        
        return result.tolist()
    
    def _update_orientation(self):
        """
        Estimate orientation based on recent position history.
        Updates the current_orientation value in degrees (0-359).
        """
        if len(self.position_history) < 2:
            return
        
        # Get the most recent positions
        current = self.position_history[-1]
        previous = self.position_history[-2]
        
        # Calculate the angle between them
        dx = current['x'] - previous['x']
        dy = current['y'] - previous['y']
        
        # Only update if there's significant movement
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            return
        
        # Calculate angle in degrees (0 = East, 90 = North, etc.)
        angle = math.degrees(math.atan2(dy, dx))
        
        # Adjust to standard bearing (0 = North, clockwise)
        bearing = (90 - angle) % 360
        
        self.current_orientation = bearing
        logger.debug(f"Orientation updated to {bearing:.1f}°")
    
    def get_current_position(self):
        """
        Get the current estimated position and orientation.
        
        Returns:
            Dictionary with position and orientation information
        """
        with self.update_lock:
            return {
                'x': self.current_position['x'],
                'y': self.current_position['y'],
                'orientation': self.current_orientation,
                'last_update': self.last_update.isoformat(),
                'confidence': self._calculate_confidence()
            }
    
    def _calculate_confidence(self):
        """
        Calculate a confidence score for the current position estimate.
        
        Returns:
            Confidence value between 0.0 and 1.0
        """
        visible_beacons = self.beacon_scanner.get_visible_beacons()
        
        # Factors affecting confidence:
        # 1. Number of visible beacons
        # 2. Time since last update
        # 3. Signal strength of beacons
        
        # Start with base confidence
        confidence = 0.5
        
        # Adjust for number of beacons
        beacon_count = len(visible_beacons)
        if beacon_count == 0:
            return 0.0
        elif beacon_count == 1:
            confidence *= 0.6
        elif beacon_count == 2:
            confidence *= 0.8
        elif beacon_count >= 3:
            confidence *= 1.0 + min((beacon_count - 3) * 0.05, 0.2)  # Max +20% for many beacons
        
        # Adjust for time since last update
        time_diff = (datetime.now() - self.last_update).total_seconds()
        if time_diff > 60:  # Stale data
            confidence *= 0.5
        elif time_diff > 30:
            confidence *= 0.7
        elif time_diff > 10:
            confidence *= 0.9
        
        # Adjust for signal strength (using average RSSI)
        avg_rssi = sum(data['rssi'] for _, data in visible_beacons.items()) / beacon_count
        if avg_rssi > -60:  # Very strong
            confidence *= 1.2
        elif avg_rssi > -70:  # Strong
            confidence *= 1.1
        elif avg_rssi > -80:  # Moderate
            confidence *= 1.0
        elif avg_rssi > -90:  # Weak
            confidence *= 0.8
        else:  # Very weak
            confidence *= 0.6
        
        # Ensure confidence is between 0 and 1
        return max(0.0, min(1.0, confidence))
    
    def handle_beacon_scan(self, scan_data):
        """
        Handle beacon scan completion event.
        
        Args:
            scan_data: Dictionary of visible beacons
        """
        self.update_position()
    
    def handle_position_request(self, data=None):
        """
        Handle request for position update.
        
        Args:
            data: Optional parameters
        """
        updated = self.update_position()
        self.event_bus.publish('position_response', {
            'success': updated,
            'position': self.get_current_position()
        })