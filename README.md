# Autonomous Navigation Robot

This is a complete autonomous navigation system for a small robot using BLE beacons for localization and ultrasonic sensors for obstacle avoidance.

## System Overview

The system uses a Raspberry Pi for high-level processing and an Arduino for low-level motor control and sensor reading. It implements:

- BLE beacon-based positioning
- Grid-based navigation with A\* pathfinding
- Obstacle detection and avoidance
- Web-based control interface

## Hardware Requirements

- **Raspberry Pi** (3 or newer)
- **Arduino** (Uno or similar)
- **Motor Controller Shield** (for Arduino)
- **DC Motors** (2 or 4 depending on chassis)
- **Ultrasonic Sensors** (HC-SR04)
- **IR Sensors** (for cliff detection and rear obstacle detection)
- **BLE Beacons** (iBeacon compatible)
- **Battery** (for powering the robot)

## Software Requirements

- Python 3.6+
- Flask
- PySerial
- NumPy
- Bleak (for BLE scanning)
- SQLite3

## Installation

1. Clone this repository:

   ```
   git clone https://github.com/yourusername/autonomous-robot.git
   cd autonomous-robot
   ```

2. Install Python dependencies:

   ```
   pip install -r requirements.txt
   ```

3. Initialize the database:

   ```
   ./start_robot.sh --init-db --import-beacons config/sample_beacons.json
   ```

4. Upload the Arduino code:
   ```
   arduino-cli upload -p /dev/ttyUSB0 -b arduino:avr:uno arduino/robot_controller.ino
   ```

## Usage

### Starting the Robot

```
./start_robot.sh [options]
```

Options:

- `-d, --demo`: Run in demo mode (no hardware connection)
- `-p, --port PORT`: Specify Arduino serial port
- `-b, --baudrate RATE`: Specify serial baudrate
- `-c, --config PATH`: Specify config file path
- `--init-db`: Initialize/reset the database
- `--import-beacons FILE`: Import beacons from a JSON file

### Web Interface

Once the system is running, access the web interface at:

```
http://raspberry-pi-ip:5000
```

The web interface allows:

- Manual robot control
- Beacon map visualization
- Navigation commands
- Sensor data visualization
- System configuration

## System Architecture

### Components

1. **BLE Positioning Module**

   - Scans for BLE beacons
   - Estimates robot position using signal strength

2. **Navigation Planner**

   - Maintains a grid map
   - Finds optimal paths using A\* algorithm
   - Generates waypoints for navigation

3. **Motion Controller**

   - Translates waypoints to motor commands
   - Communicates with Arduino

4. **Sensor Fusion**

   - Processes ultrasonic and IR sensor data
   - Detects obstacles and unsafe conditions

5. **System Coordinator**
   - Manages system state
   - Coordinates communication between modules
   - Handles event propagation

### Communication Protocol

The system uses a simple serial protocol to communicate with the Arduino:

- `Fxxx`: Move forward at speed xxx (0-255)
- `Bxxx`: Move backward at speed xxx
- `Lxxx`: Turn left at speed xxx
- `Rxxx`: Turn right at speed xxx
- `S`: Stop all motors
- `D`: Request sensor data
- `O`: Online check
- `M0/M1`: Set mode (manual/autonomous)

## Configuration

The system configuration is stored in `config/config.ini` and can be modified to adjust:

- BLE scanning parameters
- Navigation settings
- Safety thresholds
- Web interface options

## Extending the System

### Adding New Sensors

1. Modify the Arduino code to read from the new sensor
2. Update the serial communication protocol
3. Add a new sensor manager in the `modules/sensor_fusion` directory
4. Integrate the new sensor data into the obstacle detector

### Custom Navigation Algorithms

The path planning algorithm can be replaced by modifying `modules/nav_planner/path_planner.py`.

## Troubleshooting

### Common Issues

1. **Arduino not responding**

   - Check the serial port settings
   - Verify the Arduino has the correct code uploaded
   - Check power supply to Arduino

2. **BLE beacons not detected**

   - Ensure BLE is enabled on the Raspberry Pi
   - Check beacon batteries
   - Verify beacon UUIDs in the configuration

3. **Robot not navigating correctly**
   - Calibrate the positioning system
   - Check motor connections
   - Verify sensor readings

### Logs

System logs are stored in `robot.log` and contain detailed diagnostic information.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
