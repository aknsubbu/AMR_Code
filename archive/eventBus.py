import logging
import threading
import time
from typing import Dict, List, Callable, Any

class EventBus:
    """
    Simple event bus for inter-module communication.
    Supports publishing and subscribing to events.
    """
    
    def __init__(self, logger=None):
        """
        Initialize the event bus.
        
        Args:
            logger: Logger instance (optional)
        """
        self.logger = logger or logging.getLogger("EventBus")
        self._handlers = {}  # event_type -> [handlers]
        self._lock = threading.Lock()
        self._event_counters = {}  # event_type -> count
        
        self.logger.info("EventBus initialized")
    
    def register(self, event_type: str, handler: Callable):
        """
        Register a handler for an event type.
        
        Args:
            event_type: Event type string
            handler: Callback function for the event
            
        Returns:
            Success flag
        """
        with self._lock:
            if event_type not in self._handlers:
                self._handlers[event_type] = []
                
            # Avoid duplicate handlers
            if handler not in self._handlers[event_type]:
                self._handlers[event_type].append(handler)
                self.logger.debug(f"Registered handler for event: {event_type}")
                return True
            else:
                self.logger.warning(f"Handler already registered for event: {event_type}")
                return False
    
    def unregister(self, event_type: str, handler: Callable):
        """
        Unregister a handler for an event type.
        
        Args:
            event_type: Event type string
            handler: Callback function to unregister
            
        Returns:
            Success flag
        """
        with self._lock:
            if event_type in self._handlers and handler in self._handlers[event_type]:
                self._handlers[event_type].remove(handler)
                self.logger.debug(f"Unregistered handler for event: {event_type}")
                
                # Clean up empty handler lists
                if not self._handlers[event_type]:
                    del self._handlers[event_type]
                    
                return True
            else:
                self.logger.warning(f"Handler not found for event: {event_type}")
                return False
    
    def unregister_all(self, event_type: str = None):
        """
        Unregister all handlers for an event type.
        If event_type is None, unregister all handlers for all events.
        
        Args:
            event_type: Event type string (optional)
            
        Returns:
            Number of handlers unregistered
        """
        with self._lock:
            if event_type is not None:
                # Unregister for specific event type
                if event_type in self._handlers:
                    count = len(self._handlers[event_type])
                    del self._handlers[event_type]
                    self.logger.debug(f"Unregistered {count} handlers for event: {event_type}")
                    return count
                else:
                    return 0
            else:
                # Unregister all
                count = sum(len(handlers) for handlers in self._handlers.values())
                self._handlers.clear()
                self.logger.debug(f"Unregistered all {count} handlers")
                return count
    
    def publish(self, event_type: str, data: Any = None):
        """
        Publish an event to all registered handlers.
        
        Args:
            event_type: Event type string
            data: Event data (optional)
            
        Returns:
            Number of handlers notified
        """
        handlers = []
        
        # Get handlers while holding the lock
        with self._lock:
            if event_type in self._handlers:
                handlers = self._handlers[event_type].copy()
                
            # Update event counter
            if event_type not in self._event_counters:
                self._event_counters[event_type] = 0
            self._event_counters[event_type] += 1
        
        # Call handlers outside the lock
        count = 0
        for handler in handlers:
            try:
                handler(data)
                count += 1
            except Exception as e:
                self.logger.error(f"Error in event handler for {event_type}: {e}")
                
        if count > 0:
            self.logger.debug(f"Published event {event_type} to {count} handlers")
            
        return count
    
    def get_event_stats(self):
        """
        Get statistics about registered events.
        
        Returns:
            Dict with event statistics
        """
        with self._lock:
            stats = {
                'events': len(self._handlers),
                'total_handlers': sum(len(handlers) for handlers in self._handlers.values()),
                'event_counts': self._event_counters.copy(),
                'handlers_per_event': {event: len(handlers) for event, handlers in self._handlers.items()}
            }
            return stats