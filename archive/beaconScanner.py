import asyncio
import logging
import random
import time
from typing import Dict, List, Optional, Any

class BeaconScanner:
    """
    Scanner for Bluetooth beacons.
    Handles discovery and signal strength (RSSI) measurements.
    """
    
    def __init__(self, adapter_id=0, event_bus=None, logger=None, config=None):
        """
        Initialize the beacon scanner.
        
        Args:
            adapter_id: Bluetooth adapter ID
            event_bus: Optional event bus for notifications
            logger: Logger instance (optional)
            config: Configuration parameters (optional)
        """
        self.adapter_id = adapter_id
        self.event_bus = event_bus
        self.logger = logger or logging.getLogger("BeaconScanner")
        
        # Default configuration
        self.config = {
            'scan_interval': 1.0,       # Interval between scans (seconds)
            'scan_duration': 0.5,       # Duration of each scan (seconds)
            'cache_timeout': 30.0,      # How long to cache beacon data (seconds)
            'rssi_min_threshold': -85,  # Minimum RSSI threshold for detection
            'filter_duplicates': True,  # Filter duplicate advertisements
        }
        
        # Update with provided config
        if config:
            self.config.update(config)
        
        # Cached beacon data
        self._beacons = {}  # beacon_id -> {'rssi': value, 'timestamp': time}
        self._scan_lock = asyncio.Lock()
        self._running = False
        self._scan_task = None
        
        self.logger.info(f"BeaconScanner initialized with adapter {adapter_id}")
    
    async def start_scanning(self):
        """
        Start continuous background scanning.
        
        Returns:
            Success flag
        """
        async with self._scan_lock:
            if self._running:
                return True
                
            self._running = True
            
        # Start background scan task
        self._scan_task = asyncio.create_task(self._continuous_scan())
        
        self.logger.info("Beacon scanning started")
        return True
    
    async def stop_scanning(self):
        """
        Stop continuous background scanning.
        
        Returns:
            Success flag
        """
        async with self._scan_lock:
            if not self._running:
                return True
                
            self._running = False
            
        # Cancel scan task if running
        if self._scan_task and not self._scan_task.done():
            self._scan_task.cancel()
            try:
                await self._scan_task
            except asyncio.CancelledError:
                pass
                
        self.logger.info("Beacon scanning stopped")
        return True
    
    async def _continuous_scan(self):
        """
        Task for continuous background scanning.
        """
        try:
            while self._running:
                await self._perform_scan()
                await asyncio.sleep(self.config['scan_interval'])
        except asyncio.CancelledError:
            self.logger.info("Continuous scanning cancelled")
            raise
        except Exception as e:
            self.logger.error(f"Error in continuous scanning: {e}")
    
    async def _perform_scan(self):
        """
        Perform a single scan for beacons.
        
        Returns:
            Dict of discovered beacons (beacon_id -> rssi)
        """
        try:
            # This is where you would integrate with the actual BLE hardware
            # For demonstration, we'll simulate some beacon discoveries
            
            # In a real implementation, you would:
            # 1. Start a BLE scan
            # 2. Collect advertisements for the scan duration
            # 3. Filter for beacon advertisements
            # 4. Extract beacon IDs and RSSI values
            
            # Simulated scan
            await asyncio.sleep(self.config['scan_duration'])
            
            # Current timestamp for cache management
            current_time = time.time()
            
            # Update cached beacons (simulate some random changes)
            discovered = await self._simulated_scan()
            
            # Update cache with new readings
            for beacon_id, rssi in discovered.items():
                self._beacons[beacon_id] = {
                    'rssi': rssi,
                    'timestamp': current_time
                }
            
            # Publish event if event bus is available
            if self.event_bus:
                self.event_bus.publish('beacon_scan_complete', {
                    'timestamp': current_time,
                    'beacons': {k: v['rssi'] for k, v in self._beacons.items()}
                })
                
            # Clean up expired cache entries
            await self._clean_cache()
            
            return {k: v['rssi'] for k, v in self._beacons.items()}
            
        except Exception as e:
            self.logger.error(f"Error performing scan: {e}")
            return {}
    
    async def _clean_cache(self):
        """
        Clean expired entries from the beacon cache.
        """
        current_time = time.time()
        timeout = self.config['cache_timeout']
        
        # Find expired entries
        expired = [
            beacon_id for beacon_id, data in self._beacons.items()
            if current_time - data['timestamp'] > timeout
        ]
        
        # Remove expired entries
        for beacon_id in expired:
            del self._beacons[beacon_id]
            
        if expired:
            self.logger.debug(f"Removed {len(expired)} expired beacons from cache")
    
    async def _simulated_scan(self) -> Dict[str, float]:
        """
        Simulate a beacon scan for testing without hardware.
        
        Returns:
            Dict of beacon_id -> rssi
        """
        # Simulate some beacons with random RSSI fluctuations
        beacons = {
            'beacon1': random.uniform(-70, -60),
            'beacon2': random.uniform(-80, -70),
            'beacon3': random.uniform(-75, -65)
        }
        
        # Randomly remove some beacons to simulate out of range
        for beacon_id in list(beacons.keys()):
            if random.random() < 0.2:  # 20% chance to "miss" a beacon
                del beacons[beacon_id]
                
        # Apply RSSI threshold
        min_rssi = self.config['rssi_min_threshold']
        beacons = {bid: rssi for bid, rssi in beacons.items() if rssi >= min_rssi}
        
        return beacons
    
    async def scan_beacons(self):
        """
        Perform an on-demand scan for beacons.
        
        Returns:
            Dict of beacon_id -> rssi
        """
        async with self._scan_lock:
            # If continuous scanning is running, just return cached data
            if self._running:
                # But first, clean expired entries
                await self._clean_cache()
                return {k: v['rssi'] for k, v in self._beacons.items()}
                
            # Otherwise, perform a single scan
            return await self._perform_scan()
    
    async def get_rssi(self, beacon_id):
        """
        Get the current RSSI for a specific beacon.
        
        Args:
            beacon_id: Beacon identifier
            
        Returns:
            RSSI value or None if beacon not found
        """
        # Check if we need to refresh the cache
        if beacon_id not in self._beacons or time.time() - self._beacons[beacon_id]['timestamp'] > self.config['cache_timeout']:
            # Perform a fresh scan
            await self.scan_beacons()
            
        # Return RSSI if available
        if beacon_id in self._beacons:
            return self._beacons[beacon_id]['rssi']
        else:
            return None
    
    def get_all_beacons(self):
        """
        Get all currently known beacons.
        
        Returns:
            Dict of beacon_id -> rssi
        """
        return {k: v['rssi'] for k, v in self._beacons.items()}
    
    async def get_strongest_beacon(self):
        """
        Get the strongest beacon currently in range.
        
        Returns:
            Tuple of (beacon_id, rssi) or (None, None) if no beacons found
        """
        # Perform a fresh scan
        beacons = await self.scan_beacons()
        
        if not beacons:
            return None, None
            
        # Find strongest
        strongest_id = max(beacons, key=beacons.get)
        return strongest_id, beacons[strongest_id]