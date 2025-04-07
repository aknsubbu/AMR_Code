"""
State Manager for controlling the robot's operational state.
"""
import logging
import threading
import time
from enum import Enum
from datetime import datetime

logger = logging.getLogger(__name__)

class RobotState(Enum):
    """Enum representing possible robot states."""
    INITIALIZING = "INITIALIZING"
    MANUAL = "MANUAL"
    AUTONOMOUS = "AUTONOMOUS"
    EMERGENCY = "EMERGENCY"
    ERROR = "ERROR"
    CALIBRATING = "CALIBRATING"
    CHARGING = "CHARGING"
    SLEEPING = "SLEEPING"

class StateManager:
    """
    Manages the operational state of the robot system.
    Controls state transitions and ensures proper state-based behavior.
    """
    
    def __init__(self, event_bus):
        """
        Initialize the state manager.
        
        Args:
            event_bus: EventBus instance for inter-module communication
        """
        self.event_bus = event_bus
        self.current_state = RobotState.INITIALIZING
        self.previous_state = None
        self.state_entered = datetime.now()
        self.state_history = []
        self.lock = threading.RLock()
        self.state_data = {}  # Additional state-specific data
        
        # Register event handlers
        self.event_bus.register('emergency_detected', self.handle_emergency)
        self.event_bus.register('error_detected', self.handle_error)
        self.event_bus.register('navigation_complete', self.handle_navigation_complete)
        
        logger.info(f"State manager initialized in {self.current_state.name} state")
    
    def set_state(self, new_state, state_data=None):
        """
        Set the current state of the robot.
        
        Args:
            new_state: New state (string or RobotState enum)
            state_data: Optional data associated with the state
            
        Returns:
            True if state transition was successful, False otherwise
        """
        with self.lock:
            # Convert string to enum if needed
            if isinstance(new_state, str):
                try:
                    new_state = RobotState[new_state]
                except KeyError:
                    logger.error(f"Invalid state: {new_state}")
                    return False
            
            # Check if valid transition
            if not self._is_valid_transition(new_state):
                logger.warning(f"Invalid state transition: {self.current_state.name} -> {new_state.name}")
                return False
            
            # Store previous state and update current state
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_entered = datetime.now()
            self.state_data = state_data or {}
            
            # Add to history
            self.state_history.append({
                'state': new_state.name,
                'timestamp': self.state_entered.isoformat(),
                'previous': self.previous_state.name if self.previous_state else None
            })
            
            # Trim history if needed
            if len(self.state_history) > 100:
                self.state_history = self.state_history[-100:]
            
            # Notify state change
            self.event_bus.publish('state_changed', {
                'current_state': self.current_state.name,
                'previous_state': self.previous_state.name if self.previous_state else None,
                'timestamp': self.state_entered.isoformat(),
                'data': self.state_data
            })
            
            # Log the state change
            logger.info(f"State changed: {self.previous_state.name if self.previous_state else 'None'} -> {self.current_state.name}")
            
            # Perform entry actions for the new state
            self._perform_state_entry_actions()
            
            return True
    
    def _is_valid_transition(self, new_state):
        """
        Check if a state transition is valid.
        
        Args:
            new_state: Target state
            
        Returns:
            True if transition is valid, False otherwise
        """
        # Allow all transitions from INITIALIZING
        if self.current_state == RobotState.INITIALIZING:
            return True
        
        # Always allow transition to EMERGENCY
        if new_state == RobotState.EMERGENCY:
            return True
        
        # Define allowed transitions for each state
        allowed_transitions = {
            RobotState.MANUAL: [
                RobotState.AUTONOMOUS, RobotState.CALIBRATING, 
                RobotState.CHARGING, RobotState.SLEEPING, RobotState.ERROR
            ],
            RobotState.AUTONOMOUS: [
                RobotState.MANUAL, RobotState.ERROR, 
                RobotState.CHARGING, RobotState.SLEEPING
            ],
            RobotState.EMERGENCY: [
                RobotState.MANUAL, RobotState.ERROR
            ],
            RobotState.ERROR: [
                RobotState.MANUAL, RobotState.INITIALIZING
            ],
            RobotState.CALIBRATING: [
                RobotState.MANUAL, RobotState.ERROR
            ],
            RobotState.CHARGING: [
                RobotState.MANUAL, RobotState.AUTONOMOUS, 
                RobotState.SLEEPING, RobotState.ERROR
            ],
            RobotState.SLEEPING: [
                RobotState.MANUAL, RobotState.AUTONOMOUS, 
                RobotState.CHARGING, RobotState.ERROR
            ]
        }
        
        # Check if transition is allowed
        return new_state in allowed_transitions.get(self.current_state, [])
    
    def _perform_state_entry_actions(self):
        """Perform actions when entering a new state."""
        if self.current_state == RobotState.EMERGENCY:
            # Publish emergency stop command
            self.event_bus.publish('emergency_stop', {
                'reason': self.state_data.get('reason', 'Unknown')
            })
        
        elif self.current_state == RobotState.MANUAL:
            # Reset any autonomous navigation
            self.event_bus.publish('navigation_reset', None)
        
        elif self.current_state == RobotState.AUTONOMOUS:
            # Start monitoring for obstacles
            self.event_bus.publish('start_obstacle_detection', None)
        
        elif self.current_state == RobotState.SLEEPING:
            # Power down non-essential systems
            self.event_bus.publish('power_saving_mode', True)
        
        elif self.current_state == RobotState.CHARGING:
            # Disable movement
            self.event_bus.publish('disable_movement', True)
    
    def get_current_state(self):
        """
        Get the current state of the robot.
        
        Returns:
            Dictionary with state information
        """
        with self.lock:
            return {
                'state': self.current_state.name,
                'previous': self.previous_state.name if self.previous_state else None,
                'since': self.state_entered.isoformat(),
                'duration': (datetime.now() - self.state_entered).total_seconds(),
                'data': self.state_data
            }
    
    def get_state_history(self, limit=10):
        """
        Get the state transition history.
        
        Args:
            limit: Maximum number of history entries to return
            
        Returns:
            List of state transitions
        """
        with self.lock:
            return self.state_history[-limit:]
    
    def handle_emergency(self, data):
        """
        Handle emergency event.
        
        Args:
            data: Emergency event data
        """
        logger.warning(f"Emergency detected: {data}")
        self.set_state(RobotState.EMERGENCY, {'reason': data.get('reason', 'Unknown')})
    
    def handle_error(self, data):
        """
        Handle error event.
        
        Args:
            data: Error event data
        """
        logger.error(f"Error detected: {data}")
        self.set_state(RobotState.ERROR, {'error': data.get('message', 'Unknown error')})
    
    def handle_navigation_complete(self, data):
        """
        Handle navigation completion event.
        
        Args:
            data: Navigation event data
        """
        logger.info("Navigation complete")
        # Switch back to manual mode when navigation is complete
        self.set_state(RobotState.MANUAL)