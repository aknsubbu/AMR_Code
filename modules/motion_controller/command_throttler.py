"""
Command throttling system to prevent excessive serial traffic.
Provides rate limiting and command debouncing.
"""
import logging
import time
import threading
from collections import deque
from functools import wraps

logger = logging.getLogger(__name__)

class CommandThrottler:
    """
    Command throttling system that limits command rates to prevent overwhelming
    the serial connection to Arduino.
    """
    
    def __init__(self, max_rate=10, window_size=1.0, min_interval=0.05):
        """
        Initialize the command throttler.
        
        Args:
            max_rate: Maximum commands per window_size (default: 10)
            window_size: Time window in seconds (default: 1.0)
            min_interval: Minimum time between commands in seconds (default: 0.05)
        """
        self.max_rate = max_rate
        self.window_size = window_size
        self.min_interval = min_interval
        
        self.command_times = deque()
        self.command_lock = threading.RLock()
        self.last_command_time = 0
        self.command_count = 0
        
        # Command merging - track last command for each type
        self.last_commands = {}
        
        logger.info(f"Command throttler initialized (max_rate={max_rate}/sec, min_interval={min_interval*1000:.1f}ms)")
    
    def can_send_command(self, command):
        """
        Check if a command can be sent based on rate limiting.
        
        Args:
            command: Command to check
            
        Returns:
            True if command can be sent, False otherwise
        """
        with self.command_lock:
            current_time = time.time()
            
            # Check minimum interval
            if current_time - self.last_command_time < self.min_interval:
                return False
            
            # Remove old command times
            while self.command_times and self.command_times[0] < current_time - self.window_size:
                self.command_times.popleft()
            
            # Check max rate
            return len(self.command_times) < self.max_rate
    
    def record_command(self, command):
        """
        Record that a command was sent.
        
        Args:
            command: Command that was sent
        """
        with self.command_lock:
            current_time = time.time()
            self.command_times.append(current_time)
            self.last_command_time = current_time
            self.command_count += 1
            
            # Register this command for deduplication
            command_type = command[0] if command else ''
            self.last_commands[command_type] = {
                'command': command,
                'time': current_time
            }
    
    def should_deduplicate(self, command, time_threshold=0.2):
        """
        Check if a command should be deduplicated.
        
        Args:
            command: Command to check
            time_threshold: Time threshold for deduplication in seconds
            
        Returns:
            True if command should be deduplicated, False otherwise
        """
        # Empty commands are always duplicates
        if not command:
            return True
            
        command_type = command[0]
        
        with self.command_lock:
            # If we've never seen this command type, it's not a duplicate
            if command_type not in self.last_commands:
                return False
            
            last_cmd = self.last_commands[command_type]
            current_time = time.time()
            
            # If the command is identical and recent, it's a duplicate
            if last_cmd['command'] == command and current_time - last_cmd['time'] < time_threshold:
                return True
            
            return False
    
    def wait_if_needed(self, command=None):
        """
        Wait if needed to satisfy rate limiting.
        
        Args:
            command: Optional command being sent
            
        Returns:
            True if wait was successful, False if interrupted
        """
        try:
            with self.command_lock:
                current_time = time.time()
                
                # Wait for minimum interval
                time_since_last = current_time - self.last_command_time
                if time_since_last < self.min_interval:
                    wait_time = self.min_interval - time_since_last
                    time.sleep(wait_time)
                
                # Remove old command times
                while self.command_times and self.command_times[0] < current_time - self.window_size:
                    self.command_times.popleft()
                
                # Wait if at max rate
                if len(self.command_times) >= self.max_rate:
                    # Wait for oldest command to expire
                    oldest = self.command_times[0]
                    wait_time = (oldest + self.window_size) - current_time
                    if wait_time > 0:
                        time.sleep(wait_time)
            
            return True
        
        except Exception as e:
            logger.error(f"Error in command throttling wait: {e}")
            return False

    def get_stats(self):
        """
        Get statistics about command throttling.
        
        Returns:
            Dict with statistics
        """
        with self.command_lock:
            current_time = time.time()
            
            # Remove old command times for accurate count
            while self.command_times and self.command_times[0] < current_time - self.window_size:
                self.command_times.popleft()
            
            return {
                'command_count': self.command_count,
                'current_rate': len(self.command_times),
                'max_rate': self.max_rate,
                'time_since_last': current_time - self.last_command_time if self.last_command_time else None
            }

def throttle_command(throttler):
    """
    Decorator for throttling commands.
    
    Args:
        throttler: CommandThrottler instance
        
    Returns:
        Decorated function
    """
    def decorator(func):
        @wraps(func)
        def wrapper(self, command, *args, **kwargs):
            if not command:
                return "Empty command"
            
            # Check if command should be deduplicated
            if throttler.should_deduplicate(command):
                return f"Command '{command}' deduplicated (duplicate within threshold)"
            
            # Wait if needed to satisfy rate limiting
            throttler.wait_if_needed(command)
            
            # If still can't send after waiting, queue it
            if not throttler.can_send_command(command):
                logger.warning(f"Command rate limit exceeded, queuing command: {command}")
                # Let the original function handle queuing
                return func(self, command, *args, **kwargs)
            
            # Record the command
            throttler.record_command(command)
            
            # Call the original function
            return func(self, command, *args, **kwargs)
        
        return wrapper
    
    return decorator