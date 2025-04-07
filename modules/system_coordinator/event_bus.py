"""
Event Bus for inter-module communication.
"""
import logging
import threading
import queue
import time
from collections import defaultdict
from typing import Dict, List, Callable, Any

logger = logging.getLogger(__name__)

class EventBus:
    """
    Event bus for asynchronous inter-module communication.
    Allows modules to subscribe to events and publish events.
    """
    
    def __init__(self, async_dispatch=True, max_workers=4):
        """
        Initialize the event bus.
        
        Args:
            async_dispatch: Whether to dispatch events asynchronously
            max_workers: Maximum number of worker threads for event processing
        """
        self._handlers: Dict[str, List[Callable]] = defaultdict(list)
        self._async = async_dispatch
        self._max_workers = max_workers
        self._event_queue = queue.Queue()
        self._workers = []
        self._running = True
        self._lock = threading.RLock()
        
        # Start worker threads if using async dispatch
        if self._async:
            self._start_workers()
        
        logger.info(f"Event bus initialized (async={async_dispatch}, workers={max_workers})")
    
    def _start_workers(self):
        """Start worker threads for event processing."""
        for i in range(self._max_workers):
            worker = threading.Thread(target=self._event_worker, daemon=True)
            worker.start()
            self._workers.append(worker)
            logger.debug(f"Started event worker thread {i+1}")
    
    def _event_worker(self):
        """Worker thread function for processing events."""
        while self._running:
            try:
                # Get event from queue with timeout to allow checking _running
                event = self._event_queue.get(timeout=0.5)
                if event:
                    event_type, event_data = event
                    self._dispatch_event(event_type, event_data)
                self._event_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in event worker: {e}")
    
    def register(self, event_type: str, handler: Callable):
        """
        Register a handler for an event type.
        
        Args:
            event_type: Type of event to listen for
            handler: Callback function to handle the event
            
        Returns:
            True if registration was successful, False otherwise
        """
        with self._lock:
            if handler not in self._handlers[event_type]:
                self._handlers[event_type].append(handler)
                logger.debug(f"Registered handler for event type: {event_type}")
                return True
            else:
                logger.warning(f"Handler already registered for event type: {event_type}")
                return False
    
    def unregister(self, event_type: str, handler: Callable):
        """
        Unregister a handler for an event type.
        
        Args:
            event_type: Type of event
            handler: Callback function to remove
            
        Returns:
            True if unregistration was successful, False otherwise
        """
        with self._lock:
            if event_type in self._handlers and handler in self._handlers[event_type]:
                self._handlers[event_type].remove(handler)
                logger.debug(f"Unregistered handler for event type: {event_type}")
                return True
            else:
                logger.warning(f"Handler not found for event type: {event_type}")
                return False
    
    def publish(self, event_type: str, event_data: Any = None):
        """
        Publish an event to all registered handlers.
        
        Args:
            event_type: Type of event to publish
            event_data: Data associated with the event
        """
        if self._async:
            # Queue event for asynchronous processing
            self._event_queue.put((event_type, event_data))
            logger.debug(f"Queued event: {event_type}")
        else:
            # Process event synchronously
            self._dispatch_event(event_type, event_data)
    
    def _dispatch_event(self, event_type: str, event_data: Any):
        """
        Dispatch an event to all registered handlers.
        
        Args:
            event_type: Type of event to dispatch
            event_data: Data associated with the event
        """
        handlers = []
        
        # Get handlers with lock to avoid modification during iteration
        with self._lock:
            if event_type in self._handlers:
                handlers = self._handlers[event_type].copy()
        
        if not handlers:
            logger.debug(f"No handlers for event type: {event_type}")
            return
        
        # Call each handler
        for handler in handlers:
            try:
                handler(event_data)
            except Exception as e:
                logger.error(f"Error in event handler for {event_type}: {e}")
        
        logger.debug(f"Dispatched event {event_type} to {len(handlers)} handlers")
    
    def shutdown(self):
        """Shutdown the event bus and worker threads."""
        logger.info("Shutting down event bus")
        self._running = False
        
        # Wait for workers to finish
        if self._async:
            for worker in self._workers:
                worker.join(timeout=2.0)
            
            # Process any remaining events
            while not self._event_queue.empty():
                try:
                    event_type, event_data = self._event_queue.get_nowait()
                    self._dispatch_event(event_type, event_data)
                    self._event_queue.task_done()
                except queue.Empty:
                    break
                except Exception as e:
                    logger.error(f"Error processing remaining event: {e}")
        
        logger.info("Event bus shutdown complete")