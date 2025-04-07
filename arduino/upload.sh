#!/bin/bash
# Script to upload the Arduino code

# Default serial port
PORT="/dev/ttyUSB0"

# Help function
function show_help {
  echo "Usage: $0 [options]"
  echo "Upload the robot controller code to the Arduino."
  echo ""
  echo "Options:"
  echo "  -h, --help         Show this help message"
  echo "  -p, --port PORT    Specify Arduino serial port (default: $PORT)"
  echo ""
  exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
  key="$1"
  
  case $key in
    -h|--help)
      show_help
      ;;
    -p|--port)
      PORT="$2"
      shift
      shift
      ;;
    *)
      echo "Unknown option: $1"
      show_help
      ;;
  esac
done

# Check if Arduino CLI is installe
if ! command -v arduino-cli &> /dev/null; then
  echo "Arduino CLI is not installed. Please install it first."
  echo "Visit: https://arduino.github.io/arduino-cli/latest/installation/"
  exit 1
fi

echo "Compiling and uploading Arduino code to $PORT..."

# Compile and upload the code
arduino-cli compile --fqbn arduino:avr:uno robot_controller.ino && \
arduino-cli upload -p $PORT --fqbn arduino:avr:uno robot_controller.ino

if [ $? -eq 0 ]; then
  echo "Upload successful!"
else
  echo "Upload failed. Please check the Arduino connection and try again."
  exit 1
fi

echo ""
echo "Testing communication with Arduino..."
echo ""

# Wait for the Arduino to reset
sleep 2

# Test serial communication
stty -F $PORT 9600 raw -clocal -hupcl
exec 3<>$PORT

# Send online check command
echo -e "O\n" >&3

# Read response
timeout 2 cat <&3 | head -n 5

# Close file descriptor
exec 3>&-

echo ""
echo "Setup complete. The Arduino is ready."