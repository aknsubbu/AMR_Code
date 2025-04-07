#!/bin/bash
# Startup script for the autonomous navigation robot

# Default settings
DB_PATH="database/robot_db.sqlite"
CONFIG_PATH="config/config.ini"
LOG_PATH="logs/robot.log"
DEMO_MODE=false
SERIAL_PORT=""
BAUDRATE=""

# Function to display help
function show_help {
  echo "Usage: $0 [options]"
  echo "Start the autonomous navigation robot system."
  echo ""
  echo "Options:"
  echo "  -h, --help              Show this help message"
  echo "  -d, --demo              Run in demo mode (no hardware connection)"
  echo "  -p, --port PORT         Specify Arduino serial port"
  echo "  -b, --baudrate RATE     Specify serial baudrate"
  echo "  -c, --config PATH       Specify config file path"
  echo "  --init-db               Initialize/reset the database"
  echo "  --import-beacons FILE   Import beacons from a JSON file"
  echo ""
  exit 0
}

# Parse arguments
INIT_DB=false
IMPORT_BEACONS=""

while [[ $# -gt 0 ]]; do
  key="$1"
  
  case $key in
    -h|--help)
      show_help
      ;;
    -d|--demo)
      DEMO_MODE=true
      shift
      ;;
    -p|--port)
      SERIAL_PORT="$2"
      shift
      shift
      ;;
    -b|--baudrate)
      BAUDRATE="$2"
      shift
      shift
      ;;
    -c|--config)
      CONFIG_PATH="$2"
      shift
      shift
      ;;
    --init-db)
      INIT_DB=true
      shift
      ;;
    --import-beacons)
      IMPORT_BEACONS="$2"
      shift
      shift
      ;;
    *)
      echo "Unknown option: $1"
      show_help
      ;;
  esac
done

# Create directories if they don't exist
mkdir -p $(dirname "$DB_PATH")
mkdir -p $(dirname "$LOG_PATH")
mkdir -p $(dirname "$CONFIG_PATH")

# Initialize database if requested
if $INIT_DB; then
  echo "Initializing database..."
  if [ -n "$IMPORT_BEACONS" ]; then
    python3 tools/init_database.py --db "$DB_PATH" --beacons "$IMPORT_BEACONS" --force
  else
    python3 tools/init_database.py --db "$DB_PATH" --force
  fi
  
  if [ $? -ne 0 ]; then
    echo "Failed to initialize database. Exiting."
    exit 1
  fi
elif [ -n "$IMPORT_BEACONS" ]; then
  echo "Importing beacons..."
  python3 tools/init_database.py --db "$DB_PATH" --beacons "$IMPORT_BEACONS"
  
  if [ $? -ne 0 ]; then
    echo "Failed to import beacons. Exiting."
    exit 1
  fi
fi

# Build command line arguments
CMD="python3 main.py --config $CONFIG_PATH"

if $DEMO_MODE; then
  CMD="$CMD --demo"
fi

if [ -n "$SERIAL_PORT" ]; then
  CMD="$CMD --port $SERIAL_PORT"
fi

if [ -n "$BAUDRATE" ]; then
  CMD="$CMD --baudrate $BAUDRATE"
fi

# Start the system
echo "Starting robot system..."
echo "Command: $CMD"
eval $CMD