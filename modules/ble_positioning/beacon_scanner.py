"""
Beacon Scanner module for detecting and processing BLE beacons.
"""
import asyncio
import logging
import time
import threading
from bleak import BleakScanner
from bleak.backends.scanner import AdvertisementData
from datetime import datetime

logger = logging.getLogger(__name__)

class BeaconScanner:
    """
    Scans for BLE beacons and processes the detected signals.
    Uses Bleak for cross-platform BLE scanning.
    """
    
    def __init__(self, event_bus, scan_interval=1.0):
        """
        Initialize the beacon scanner.
        
        Args:
            event_bus: Event bus for inter-module communication
            scan_interval: Time between scans in seconds
        """
        self.event_bus = event_bus
        self.scan_interval = float(scan_interval)
        self.scanning = False
        self.scan_thread = None
        self.beacons = {}  # name -> {rssi, last_seen, count, mac_address}
        self._lock = threading.Lock()
        
        # Register event handlers
        self.event_bus.register('request_scan', self.handle_scan_request)
        
        logger.info("Beacon scanner initialized")
    
    def start_scanning(self):
        """Start the background scanning thread."""
        if self.scanning:
            logger.warning("Scanner already running")
            return
        
        self.scanning = True
        self.scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
        self.scan_thread.start()
        logger.info("BLE scanning started")
    
    def stop_scanning(self):
        """Stop the background scanning thread."""
        self.scanning = False
        if self.scan_thread:
            self.scan_thread.join(timeout=2.0)
        logger.info("BLE scanning stopped")
    
    def _scan_loop(self):
        """Background thread for continuous scanning."""
        while self.scanning:
            try:
                # Run the async scan in a new event loop
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                loop.run_until_complete(self._scan_once())
                loop.close()
                
                # Publish scan results
                self.event_bus.publish('beacon_scan_complete', self.get_visible_beacons())
                
                # Wait for next scan interval
                time.sleep(self.scan_interval)
            except Exception as e:
                logger.error(f"Error in scan loop: {e}")
                time.sleep(1.0)  # Short delay to avoid tight loop on error
    
    async def _scan_once(self):
        """Perform a single BLE scan."""
        logger.debug("Starting BLE scan...")
        
        try:
            # Use discovery with the detection callback
            devices = await BleakScanner.discover(timeout=3.0)
            
            timestamp = datetime.now()
            discovered = []
            
            with self._lock:
                for device in devices:
                    try:
                        # Extract device info
                        mac_address = device.address
                        name = device.name
                        
                        # Skip devices without names
                        if not name or name.lower() == "unknown":
                            if hasattr(device, 'metadata') and device.metadata.get('uuids'):
                                # Fall back to UUID if needed but log the situation
                                logger.debug(f"Device {mac_address} has no name but has UUIDs: {device.metadata.get('uuids')}")
                            continue
                        
                        # Get RSSI value, ensure it's a float
                        rssi = -100.0  # Default value if RSSI not available
                        
                        if hasattr(device, 'rssi'):
                            try:
                                rssi = float(device.rssi)
                            except (ValueError, TypeError):
                                logger.warning(f"Invalid RSSI value for {name}: {device.rssi}")
                                
                        # Use the device name as the primary identifier
                        beacon_id = name
                        
                        if beacon_id not in self.beacons:
                            self.beacons[beacon_id] = {
                                'rssi': rssi,
                                'last_seen': timestamp,
                                'count': 1,
                                'mac_address': mac_address
                            }
                            logger.debug(f"New beacon detected: {beacon_id} ({mac_address})")
                        else:
                            # Verify MAC address matches
                            if self.beacons[beacon_id]['mac_address'] != mac_address:
                                logger.warning(f"MAC address changed for {beacon_id}: {self.beacons[beacon_id]['mac_address']} -> {mac_address}")
                                self.beacons[beacon_id]['mac_address'] = mac_address
                            
                            # Update existing beacon
                            self.beacons[beacon_id]['rssi'] = self._smooth_rssi(
                                self.beacons[beacon_id]['rssi'], 
                                rssi,
                                self.beacons[beacon_id]['count']
                            )
                            self.beacons[beacon_id]['last_seen'] = timestamp
                            self.beacons[beacon_id]['count'] += 1
                        
                        discovered.append(beacon_id)
                    except Exception as e:
                        logger.error(f"Error processing device {getattr(device, 'address', 'unknown')}: {e}")
                        continue
                
                # Log when beacons disappear
                current_beacons = set(self.beacons.keys())
                seen_beacons = set(discovered)
                for disappeared in current_beacons - seen_beacons:
                    # If not seen for more than 30 seconds, mark as inactive
                    last_seen = self.beacons[disappeared]['last_seen']
                    if (timestamp - last_seen).total_seconds() > 30:
                        logger.debug(f"Beacon {disappeared} not seen recently")
            
            logger.debug(f"Scan complete. Found {len(discovered)} beacons")
            return discovered
            
        except Exception as e:
            logger.error(f"Error during BLE scan: {e}")
            return []
    
    def _smooth_rssi(self, old_rssi, new_rssi, count, alpha=0.2):
        """
        Apply exponential smoothing to RSSI values.
        
        Args:
            old_rssi: Previous RSSI value
            new_rssi: New RSSI value
            count: Number of readings so far
            alpha: Smoothing factor (0-1)
            
        Returns:
            Smoothed RSSI value
        """
        try:
            old_rssi = float(old_rssi)
            new_rssi = float(new_rssi)
            count = int(count)
            alpha = float(alpha)
            
            if count <= 1:
                return new_rssi
            return old_rssi * (1 - alpha) + new_rssi * alpha
        except (ValueError, TypeError) as e:
            logger.error(f"Error in RSSI smoothing: {e}, using new value directly")
            try:
                return float(new_rssi)
            except (ValueError, TypeError):
                return -100.0  # Default fallback
    
    def get_visible_beacons(self):
        """
        Get currently visible beacons with their RSSI values.
        
        Returns:
            Dict of {name: {rssi, mac_address}} for visible beacons
        """
        now = datetime.now()
        recent_beacons = {}
        
        with self._lock:
            for name, data in self.beacons.items():
                # Only include beacons seen in the last 30 seconds
                if (now - data['last_seen']).total_seconds() < 30:
                    recent_beacons[name] = {
                        'rssi': data['rssi'],
                        'mac_address': data['mac_address']
                    }
        
        return recent_beacons
    
    def handle_scan_request(self, data=None):
        """
        Handle a request to perform an immediate scan.
        
        Args:
            data: Optional parameters for the scan
        """
        logger.info("Immediate scan requested")
        # Run an async scan in a new thread
        thread = threading.Thread(target=self._run_immediate_scan, daemon=True)
        thread.start()
    
    def _run_immediate_scan(self):
        """Run an immediate scan in a new thread."""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            loop.run_until_complete(self._scan_once())
            loop.close()
            # Publish the results
            self.event_bus.publish('beacon_scan_complete', self.get_visible_beacons())
        except Exception as e:
            logger.error(f"Error during immediate scan: {e}")