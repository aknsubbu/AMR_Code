/*
 * Robot Controller for Autonomous Navigation System (Optimized Version)
 * 
 * This Arduino sketch handles:
 * - Motor control
 * - IR sensor reading
 * - Serial communication with Raspberry Pi
 * 
 * Ultrasonic sensors have been moved to Raspberry Pi GPIO.
 * 
 * Hardware Configuration:
 * - IR Sensor 1 (Back): PIN 7
 * - IR Sensor 2 (Cliff): PIN 8
 * - Motor A: Forward PIN 5, Backward PIN 3
 * - Motor B: Forward PIN 6, Backward PIN 11
 */
// Motor pin definitions
const int MOTOR_A1 = 5;  // Motor A Forward
const int MOTOR_A2 = 3;  // Motor A Backward
const int MOTOR_B1 = 6;  // Motor B Forward 
const int MOTOR_B2 = 11; // Motor B Backward

// IR Sensor pins
const int IR_SENSOR_BACK = 7;  // Back obstacle sensor
const int IR_SENSOR_CLIFF = 8; // Cliff detection sensor

// Safety constants
const int SAFE_DISTANCE_CM = 15;  // Safe distance in centimeters

// Operating mode (0 = manual, 1 = autonomous)
int operatingMode = 0;

// Command buffer
String command;

// Timing variables
unsigned long lastSensorCheck = 0;
unsigned long lastSerialSend = 0;
unsigned long lastCommandProcess = 0;
const int SENSOR_CHECK_INTERVAL = 2000;  // Check sensors every 200ms when moving
const int SERIAL_SEND_INTERVAL = 500;   // Limit serial transmission frequency
const int COMMAND_PROCESS_INTERVAL = 50; // Avoid processing commands too frequently

// Set to true to ignore cliff sensor readings (for testing)
const bool IGNORE_CLIFF_SENSOR = false;

void setup() {
  // Set motor pins as outputs
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  // Set IR sensor pins as inputs
  pinMode(IR_SENSOR_BACK, INPUT);
  pinMode(IR_SENSOR_CLIFF, INPUT);
  
  // Start serial communication
  Serial.begin(9600);
  delay(100);
  
  // Initialization complete message
  Serial.println("Robot Controller v3.0");
  Serial.println("Motors and IR Sensors Initialized");
  Serial.println("Commands: F/B/L/R/S (direction) + speed (0-255)");
  Serial.println("Special: O (online check), D (data request), M (mode)");
}

void loop() {
  // Prevent processing too many commands in rapid succession
  unsigned long currentMillis = millis();
  
  // Check for emergency conditions first (at controlled intervals)
  if (currentMillis - lastSensorCheck >= SENSOR_CHECK_INTERVAL) {
    checkSensors();
    lastSensorCheck = currentMillis;
  }
  
  // Check if serial data is available (but not too frequently)
  if (Serial.available() > 0 && currentMillis - lastCommandProcess >= COMMAND_PROCESS_INTERVAL) {
    command = Serial.readStringUntil('\n'); // Read until newline
    executeCommand(command);
    lastCommandProcess = currentMillis;
  }
}

void checkSensors() {
  // Check all safety sensors
  bool emergencyStop = false;
  String reason = "";
  
  // Check cliff sensor (only if not ignored)
  if (!IGNORE_CLIFF_SENSOR && !digitalRead(IR_SENSOR_CLIFF)) {
    emergencyStop = true;
    reason = "Cliff Detected";
  }
  
  // Check rear obstacle
  if (digitalRead(IR_SENSOR_BACK)) {
    emergencyStop = true;
    reason = "Obstacle at the back";
  }
  
  // If any emergency condition detected
  if (emergencyStop) {
    stop();
    
    // Only send if it's been long enough since last send
    unsigned long currentMillis = millis();
    if (currentMillis - lastSerialSend >= SERIAL_SEND_INTERVAL) {
      Serial.println(reason);
      sendSensorData();
      lastSerialSend = currentMillis;
    }
  }
}

void sendSensorData() {
  // Send IR sensor readings only
  Serial.print("IR:");
  Serial.print(digitalRead(IR_SENSOR_BACK));
  Serial.print(",");
  Serial.println(digitalRead(IR_SENSOR_CLIFF));
}

// Execute motor commands based on serial input
void executeCommand(String command) {
  if (command.length() < 1) {
    Serial.println("Invalid command: Empty");
    return;
  }
  
  char firstChar = command.charAt(0);
  
  // Special commands
  if (firstChar == 'O' || firstChar == 'o') {
    // Online check
    Serial.println("Online");
    return;
  }
  else if (firstChar == 'D' || firstChar == 'd') {
    // Data request (send minimal data for efficiency)
    sendSensorData();
    return;
  }
  else if (firstChar == 'M' || firstChar == 'm') {
    // Mode change
    if (command.length() > 1) {
      int newMode = command.charAt(1) - '0';
      if (newMode == 0 || newMode == 1) {
        operatingMode = newMode;
        Serial.print("M:");
        Serial.println(operatingMode);
      } else {
        Serial.println("E:Invalid mode");
      }
    } else {
      Serial.print("M:");
      Serial.println(operatingMode);
    }
    return;
  }
  
  // Movement commands
  char direction = firstChar;
  int speed = 128; // Default speed if not specified
  
  // Extract speed if present
  if (command.length() > 1) {
    speed = command.substring(1).toInt();
    speed = constrain(speed, 0, 255); // Ensure speed is valid
  }
  
  // Execute appropriate movement
  switch(direction) {
    case 'F':
    case 'f':
      forward(speed);
      Serial.println("A:F"); // Acknowledge - Forward
      break;
      
    case 'B':
    case 'b':
      backward(speed);
      Serial.println("A:B"); // Acknowledge - Backward
      break;
      
    case 'L':
    case 'l':
      turnLeft(speed);
      Serial.println("A:L"); // Acknowledge - Left
      break;
      
    case 'R':
    case 'r':
      turnRight(speed);
      Serial.println("A:R"); // Acknowledge - Right
      break;
      
    case 'S':
    case 's':
      stop();
      Serial.println("A:S"); // Acknowledge - Stop
      break;
      
    default:
      Serial.println("E:Unknown command");
  }
}

// Motor control functions
void forward(int speed) {
  analogWrite(MOTOR_A1, speed);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, speed);
  analogWrite(MOTOR_B2, 0);
}

void backward(int speed) {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, speed);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, speed);
}

void turnLeft(int speed) {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, speed);
  analogWrite(MOTOR_B1, speed);
  analogWrite(MOTOR_B2, 0);
}

void turnRight(int speed) {
  analogWrite(MOTOR_A1, speed);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, speed);
}

void stop() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}