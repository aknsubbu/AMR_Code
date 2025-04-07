#!/usr/bin/env python3
"""
Main entry point for the autonomous navigation robot system.
Initializes all modules and starts the web server.
Updated to use name-based beacon identification.
"""
import os
import logging
import threading
import argparse
import time
from datetime import datetime
from flask import Flask, request, jsonify, render_template, send_from_directory
from pathlib import Path

# Import modules
from modules.ble_positioning.beacon_scanner import BeaconScanner
from modules.ble_positioning.beacon_registry import BeaconRegistry
from modules.ble_positioning.position_estimator import PositionEstimator

from modules.nav_planner.grid_map import GridMap
from modules.nav_planner.path_planner import PathPlanner
from modules.nav_planner.waypoint_generator import WaypointGenerator

from modules.sensor_fusion.ultrasonic_manager import UltrasonicManager
from modules.sensor_fusion.ir_sensor_manager import IRSensorManager
from modules.sensor_fusion.obstacle_detector import ObstacleDetector

from modules.motion_controller.command_translator import CommandTranslator
from modules.motion_controller.serial_communicator import SerialCommunicator

from modules.system_coordinator.state_manager import StateManager, RobotState
from modules.system_coordinator.event_bus import EventBus
from modules.system_coordinator.config_loader import ConfigLoader

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("robot.log"),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

# Flask app
app = Flask(__name__, 
            template_folder='web_interface/templates',
            static_folder='web_interface/static')

# Message buffer for UI
message_buffer = []

# Parse command line arguments
parser = argparse.ArgumentParser(description='Autonomous Navigation Robot')
parser.add_argument('--demo', action='store_true', help='Run in demo mode without hardware')
parser.add_argument('--config', type=str, default='config/config.ini', help='Configuration file')
parser.add_argument('--port', type=str, help='Arduino serial port (overrides config)')
parser.add_argument('--baudrate', type=int, help='Serial baudrate (overrides config)')
args = parser.parse_args()

# Global variables for system components
config_loader = None
beacon_registry = None
beacon_scanner = None
position_estimator = None
grid_map = None
path_planner = None
waypoint_generator = None
ultrasonic_manager = None
ir_sensor_manager = None
obstacle_detector = None
command_translator = None
serial_communicator = None
state_manager = None
event_bus = None

def log_message(message, level="INFO"):
    """Add a message to the UI message buffer and log it."""
    message_buffer.append(message)
    
    # Keep buffer size limited
    if len(message_buffer) > 100:
        message_buffer.pop(0)
    
    # Log based on level
    if level == "ERROR":
        logger.error(message)
    elif level == "WARNING":
        logger.warning(message)
    else:
        logger.info(message)

def event_handler(event_type, data=None):
    """Generic event handler for logging important events to UI."""
    if event_type == 'obstacle_detected':
        if data and 'message' in data:
            log_message(f"Obstacle detected: {data['message']}", "WARNING")
    
    elif event_type == 'path_found':
        if data and 'path_length' in data:
            log_message(f"Path found with {data['path_length']} steps")
    
    elif event_type == 'path_not_found':
        log_message("No path found to destination", "WARNING")
    
    elif event_type == 'waypoints_generated':
        if data and 'count' in data:
            log_message(f"Generated {data['count']} waypoints")
    
    elif event_type == 'waypoint_executed':
        if data and 'waypoint' in data and 'type' in data['waypoint']:
            waypoint_type = data['waypoint']['type']
            log_message(f"Executed {waypoint_type} waypoint")
    
    elif event_type == 'state_changed':
        if data and 'current_state' in data:
            log_message(f"Robot state changed to {data['current_state']}")

def initialize_system():
    """Initialize all system components"""
    global config_loader, beacon_registry, beacon_scanner, position_estimator
    global grid_map, path_planner, waypoint_generator
    global ultrasonic_manager, ir_sensor_manager, obstacle_detector
    global command_translator, serial_communicator
    global state_manager, event_bus
    
    log_message("Initializing autonomous navigation system...")
    
    # Initialize Configuration Loader
    config_loader = ConfigLoader(args.config)
    
    # Initialize Event Bus (for inter-module communication)
    event_bus = EventBus()
    
    # Register generic event handler
    event_types = [
        'obstacle_detected', 'path_found', 'path_not_found',
        'waypoints_generated', 'waypoint_executed', 'state_changed'
    ]
    for event_type in event_types:
        event_bus.register(event_type, lambda data, et=event_type: event_handler(et, data))
    
    # Initialize State Manager
    state_manager = StateManager(event_bus)
    
    # Initialize BLE Positioning Module
    beacon_registry = BeaconRegistry('database/robot_db.sqlite')
    
    ble_scan_interval = config_loader.get('BLE', 'scan_interval', 5.0, float)
    beacon_scanner = BeaconScanner(event_bus, scan_interval=ble_scan_interval)
    
    positioning_method = config_loader.get('BLE', 'positioning_method', 'weighted')
    position_estimator = PositionEstimator(beacon_registry, beacon_scanner, event_bus, method=positioning_method)
    
    # Initialize Navigation Planner Module
    grid_map = GridMap(beacon_registry)
    
    allow_diagonal = config_loader.get('Navigation', 'allow_diagonal', True, bool)
    path_planner = PathPlanner(grid_map, event_bus, allow_diagonal=allow_diagonal)
    
    waypoint_generator = WaypointGenerator(event_bus)
    waypoint_generator.grid_scale = config_loader.get('Navigation', 'grid_scale', 10, int)
    waypoint_generator.turn_threshold = config_loader.get('Motion', 'turn_threshold', 20, int)
    waypoint_generator.default_speed = config_loader.get('Motion', 'default_speed', 128, int)
    
    # Initialize Serial Communication
    port = args.port if args.port else config_loader.get('Serial', 'port', '/dev/ttyUSB0')
    baudrate = args.baudrate if args.baudrate else config_loader.get('Serial', 'baudrate', 9600, int)
    
    if args.demo:
        log_message("Running in DEMO mode - no hardware connection")
        serial_communicator = SerialCommunicator(None, event_bus, demo_mode=True)
    else:
        try:
            serial_communicator = SerialCommunicator(port, event_bus, 
                                                  baudrate=baudrate)
            log_message(f"Connected to Arduino on {port}")
        except Exception as e:
            log_message(f"Error connecting to Arduino: {e}", "ERROR")
            log_message("Falling back to DEMO mode", "WARNING")
            serial_communicator = SerialCommunicator(None, event_bus, demo_mode=True)
    
    # Initialize Sensor Fusion Module
    ultrasonic_manager = UltrasonicManager(serial_communicator, event_bus)
    ir_sensor_manager = IRSensorManager(serial_communicator, event_bus)
    
    obstacle_memory_time = config_loader.get('Motion', 'obstacle_memory_time', 30.0, float)
    obstacle_detector = ObstacleDetector(ultrasonic_manager, ir_sensor_manager, event_bus)
    obstacle_detector.obstacle_memory_time = obstacle_memory_time
    
    # Set grid map in obstacle detector
    event_bus.publish('set_grid_map', {'grid_map': grid_map})
    
    # Initialize Motion Controller Module
    command_translator = CommandTranslator(event_bus, default_speed=config_loader.get('Motion', 'default_speed', 128, int))
    
    # Start BLE scanning in background
    beacon_scanner.start_scanning()
    
    # Set initial state
    default_mode = config_loader.get('System', 'default_mode', 'MANUAL')
    state_manager.set_state(default_mode)
    
    log_message("System initialization complete")
    return True


# API Routes
@app.route('/')
def index():
    """Render main control interface"""
    return render_template('index.html')

@app.route('/api/status')
def get_status():
    """Get the current status of the robot"""
    return jsonify({
        'status': serial_communicator.is_connected() if serial_communicator else 'disconnected',
        'position': position_estimator.get_current_position() if position_estimator else None,
        'timestamp': datetime.now().isoformat()
    })

@app.route('/api/command', methods=['POST'])
def send_command():
    """Send manual command to the robot"""
    data = request.get_json()
    if not data or 'command' not in data:
        return jsonify({'status': 'error', 'message': 'No command provided'}), 400
    
    command = data['command']
    result = serial_communicator.send_command(command)
    
    # Log the command
    log_message(f"Sent command: {command}")
    
    return jsonify({'status': 'success', 'response': result})

@app.route('/api/robot/state')
def get_robot_state():
    """Get the current state of the robot"""
    state_data = state_manager.get_current_state() if state_manager else {'state': 'UNKNOWN'}
    
    # Add position data if available
    if position_estimator:
        state_data['position'] = position_estimator.get_current_position()
    
    return jsonify(state_data)

@app.route('/api/robot/mode', methods=['POST'])
def set_robot_mode():
    """Set the operating mode of the robot"""
    data = request.get_json()
    if not data or 'mode' not in data:
        return jsonify({'status': 'error', 'message': 'No mode provided'}), 400
    
    mode = data['mode']
    success = state_manager.set_state(mode)
    
    if success:
        log_message(f"Mode changed to {mode}")
        return jsonify({'status': 'success', 'message': f'Mode changed to {mode}'})
    else:
        return jsonify({'status': 'error', 'message': f'Invalid mode: {mode}'}), 400

@app.route('/api/beacons', methods=['GET'])
def get_beacons():
    """Get all registered beacons"""
    beacons = beacon_registry.get_all_beacons()
    return jsonify({'beacons': beacons})

@app.route('/api/beacons', methods=['POST'])
def add_beacon():
    """Add or update a beacon using name as primary identifier"""
    data = request.get_json()
    if not data or 'name' not in data or 'mac_address' not in data or 'x' not in data or 'y' not in data:
        return jsonify({'status': 'error', 'message': 'Missing required beacon data'}), 400
    
    beacon_registry.add_beacon(
        data['name'], 
        data['mac_address'], 
        data['x'], 
        data['y'], 
        data.get('description', '')
    )
    
    log_message(f"Added beacon: {data['name']} at ({data['x']}, {data['y']})")
    return jsonify({'status': 'success', 'message': 'Beacon added'})

@app.route('/api/beacons/<name>', methods=['DELETE'])
def delete_beacon(name):
    """Delete a beacon by name"""
    success = beacon_registry.remove_beacon(name)
    if success:
        log_message(f"Removed beacon: {name}")
        return jsonify({'status': 'success', 'message': 'Beacon removed'})
    else:
        return jsonify({'status': 'error', 'message': 'Beacon not found'}), 404

@app.route('/api/navigation/start', methods=['POST'])
def start_navigation():
    """Start autonomous navigation to a destination"""
    data = request.get_json()
    if not data or 'destination' not in data:
        return jsonify({'status': 'error', 'message': 'No destination provided'}), 400
    
    destination = data['destination']
    
    # Change to autonomous mode
    state_manager.set_state('AUTONOMOUS')
    
    # Request position update
    position_estimator.update_position()
    current_position = position_estimator.get_current_position()
    
    log_message(f"Starting navigation from {current_position['x']},{current_position['y']} to destination: {destination}")
    
    # Find path
    path = path_planner.find_path(current_position, destination)
    if not path:
        log_message("No path found to destination", "ERROR")
        return jsonify({'status': 'error', 'message': 'No path found'}), 404
    
    # Generate waypoints
    waypoints = waypoint_generator.generate_waypoints(path, current_position.get('orientation', 0))
    
    # Execute first waypoint (rest will follow as events)
    if waypoints:
        command_translator.execute_waypoints(waypoints)
    
    return jsonify({
        'status': 'success', 
        'message': f'Navigation started to {destination}',
        'path_length': len(path),
        'waypoints': len(waypoints)
    })

@app.route('/api/navigation/grid-click', methods=['POST'])
def grid_click_navigation():
    """Start navigation to a grid coordinate that was clicked on the map"""
    data = request.get_json()
    if not data or 'x' not in data or 'y' not in data:
        return jsonify({'status': 'error', 'message': 'No destination coordinates provided'}), 400
    
    # Get target coordinates
    target_x = int(data['x'])
    target_y = int(data['y'])
    
    # Change to autonomous mode
    state_manager.set_state('AUTONOMOUS')
    
    # Request position update
    position_estimator.update_position()
    current_position = position_estimator.get_current_position()
    
    log_message(f"Starting navigation from {current_position['x']},{current_position['y']} to grid position: ({target_x}, {target_y})")
    
    # Check if target is valid
    if not grid_map.is_navigable(target_x, target_y):
        # Try to find nearest navigable cell
        nearest = grid_map.find_nearest_navigable(target_x, target_y)
        if nearest:
            target_x, target_y = nearest
            log_message(f"Target not navigable, adjusted to nearest valid position: ({target_x}, {target_y})")
        else:
            log_message(f"Target position ({target_x}, {target_y}) is not navigable", "ERROR")
            return jsonify({'status': 'error', 'message': 'Target position is not navigable'}), 400
    
    # Find path
    destination = (target_x, target_y)
    path = path_planner.find_path(current_position, destination)
    if not path:
        log_message("No path found to destination", "ERROR")
        return jsonify({'status': 'error', 'message': 'No path found'}), 404
    
    # Generate waypoints
    waypoints = waypoint_generator.generate_waypoints(path, current_position.get('orientation', 0))
    
    # Execute waypoints
    if waypoints:
        command_translator.execute_waypoints(waypoints)
    
    # Check if clicked on a beacon
    beacon_name = grid_map.get_beacon_at_position(target_x, target_y)
    destination_desc = f"beacon '{beacon_name}'" if beacon_name else f"grid position ({target_x}, {target_y})"
    
    return jsonify({
        'status': 'success', 
        'message': f'Navigation started to {destination_desc}',
        'path_length': len(path),
        'waypoints': len(waypoints),
        'destination': {
            'x': target_x,
            'y': target_y,
            'beacon_name': beacon_name
        }
    })

@app.route('/api/navigation/stop', methods=['POST'])
def stop_navigation():
    """Stop autonomous navigation"""
    state_manager.set_state('MANUAL')
    serial_communicator.send_command('S0')  # Stop motors
    
    # Stop waypoint execution
    event_bus.publish('stop_execution', None)
    
    log_message("Navigation stopped")
    return jsonify({'status': 'success', 'message': 'Navigation stopped'})

@app.route('/api/position')
def get_position():
    """Get current position of the robot"""
    position_estimator.update_position()
    position = position_estimator.get_current_position()
    return jsonify({'position': position})

@app.route('/api/sensors')
def get_sensor_data():
    """Get current sensor readings"""
    # Get ultrasonic distances
    ultrasonic_distances = ultrasonic_manager.get_current_distances() if ultrasonic_manager else None
    
    # Get IR sensor status
    ir_status = ir_sensor_manager.get_sensor_status() if ir_sensor_manager else None
    
    # Get obstacles
    raw_obstacles = obstacle_detector.get_all_obstacles() if obstacle_detector else None
    
    # Convert tuple keys to strings for JSON serialization
    obstacles = None
    if raw_obstacles:
        obstacles = {}
        for pos, data in raw_obstacles.items():
            # Convert tuple key to string (e.g., "(0, 1)")
            if isinstance(pos, tuple):
                str_key = f"{pos[0]},{pos[1]}"
            else:
                str_key = str(pos)
            obstacles[str_key] = data
    
    # Request fresh sensor data from Arduino
    if serial_communicator:
        serial_communicator.request_sensor_data()
    
    # Format ultrasonic distances
    distances = None
    if ultrasonic_distances and ultrasonic_manager:
        # Get combined front distance (minimum of both sensors)
        front_distance = ultrasonic_manager.get_distance('front')
        
        distances = {
            'front': front_distance,  # Combined front reading
            'front1': ultrasonic_distances[0],  # Individual readings
            'front2': ultrasonic_distances[1]
        }
    
    return jsonify({
        'distances': distances,
        'ir': ir_status,
        'obstacles': obstacles,
        'timestamp': datetime.now().isoformat(),
        'messages': message_buffer.copy()  # Get a copy of the buffer
    })

@app.route('/api/settings', methods=['GET'])
def get_settings():
    """Get system settings"""
    settings = config_loader.get_all() if config_loader else {}
    return jsonify({'settings': settings})

@app.route('/api/settings', methods=['POST'])
def update_setting():
    """Update a system setting"""
    data = request.get_json()
    if not data or 'setting' not in data or 'value' not in data:
        return jsonify({'status': 'error', 'message': 'Missing required data'}), 400
    
    setting = data['setting']
    value = data['value']
    
    # Map settings to sections and options
    settings_map = {
        'posMethod': ('BLE', 'positioning_method'),
        'safeDistance': ('Motion', 'safe_distance_front'),
        'gridScale': ('Navigation', 'grid_scale')
    }
    
    if setting not in settings_map:
        return jsonify({'status': 'error', 'message': f'Unknown setting: {setting}'}), 400
    
    section, option = settings_map[setting]
    success = config_loader.set(section, option, value)
    
    if success:
        # Apply setting change immediately
        if setting == 'posMethod':
            if position_estimator:
                position_estimator.method = value
        elif setting == 'safeDistance':
            if ultrasonic_manager:
                ultrasonic_manager.safe_distances[0] = float(value)
        elif setting == 'gridScale':
            if waypoint_generator:
                waypoint_generator.grid_scale = int(value)
        
        log_message(f"Setting updated: {setting} = {value}")
        return jsonify({'status': 'success', 'message': f'Setting updated: {setting} = {value}'})
    else:
        return jsonify({'status': 'error', 'message': 'Failed to update setting'}), 500

@app.route('/api/paths')
def get_paths():
    """Get available paths and current path"""
    current_path = path_planner.last_path if path_planner else None
    
    return jsonify({
        'current_path': current_path,
        'has_path': current_path is not None
    })

@app.route('/api/waypoints')
def get_waypoints():
    """Get current waypoints"""
    current_waypoints = []
    if command_translator:
        # Get any queued waypoints
        if hasattr(command_translator, 'waypoint_queue'):
            try:
                current_waypoints = list(command_translator.waypoint_queue.queue)
            except:
                pass
    
    return jsonify({
        'waypoints': current_waypoints,
        'current_waypoint': command_translator.current_waypoint if command_translator else None,
        'executing': command_translator.executing if command_translator else False
    })

@app.route('/api/map')
def get_map_data():
    """Get grid map data"""
    if not grid_map:
        return jsonify({'status': 'error', 'message': 'Grid map not initialized'}), 500
    
    dimensions = grid_map.get_grid_dimensions()
    grid_array = grid_map.get_grid_as_array().tolist()
    beacons = {}
    
    # Convert to a simpler format for the frontend
    # Now using name as the key instead of UUID
    for pos, name in grid_map.beacons.items():
        beacons[name] = {'x': pos[0], 'y': pos[1]}
    
    return jsonify({
        'dimensions': dimensions,
        'grid': grid_array,
        'beacons': beacons,
        'clickable': True  # Indicate that the map supports click-to-navigate
    })

@app.route('/logs')
def get_logs():
    """Get system logs"""
    return jsonify({'logs': message_buffer})

@app.route('/favicon.ico')
def favicon():
    """Serve favicon"""
    return send_from_directory(os.path.join(app.root_path, 'web_interface', 'static'),
                          'favicon.ico', mimetype='image/vnd.microsoft.icon')

if __name__ == '__main__':
    # Initialize the system
    if initialize_system():
        # Get web server settings from config
        web_port = config_loader.get('Web', 'port', 5000, int)
        web_host = config_loader.get('Web', 'host', '0.0.0.0')
        web_debug = config_loader.get('Web', 'debug', True, bool)
        
        # Run the server
        log_message(f"Starting web server on {web_host}:{web_port}")
        app.run(host=web_host, port=web_port, debug=web_debug)
    else:
        logger.error("Failed to initialize system. Exiting.")