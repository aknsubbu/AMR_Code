#define START_MARKER 0xFF
#define ACK_MARKER 0xFE

// Priority handling
char currentAction = 'S';     
bool priorityActive = false;
unsigned long lastPriorityTime = 0;
const unsigned long PRIORITY_TIMEOUT = 2000;  // ms

// Motor pin definitions
const uint8_t MOTOR_A1 = 5;
const uint8_t MOTOR_A2 = 3;
const uint8_t MOTOR_B1 = 6;
const uint8_t MOTOR_B2 = 11;

// IR Sensor pins
const uint8_t IR_SENSOR_BACK = 7;
const uint8_t IR_SENSOR_CLIFF = 8;

void setup() {
  Serial.begin(2000000);  // High speed USB serial
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);
  
  pinMode(IR_SENSOR_BACK, INPUT);
  pinMode(IR_SENSOR_CLIFF, INPUT);
}

void loop() {
  handleSerialInput();
  checkPriorityTimeout();
  checkIRSensorEmergencyStop();
}

void handleSerialInput() {
  if (Serial.available() >= 3) {
    uint8_t marker = Serial.read();
    if (marker != START_MARKER) return;

    uint8_t action_byte = Serial.read();
    uint8_t speed = Serial.read();

    bool is_priority = (action_byte & 0x80) != 0;
    char action = action_byte & 0x7F;

    if (is_priority) {
      priorityActive = true;
      lastPriorityTime = millis();
      currentAction = action;
      executeAction(currentAction, speed);
    } else if (!priorityActive) {
      currentAction = action;
      executeAction(currentAction, speed);
    }
    sendAck(currentAction, speed);
  }
}

void checkPriorityTimeout() {
  if (priorityActive && millis() - lastPriorityTime > PRIORITY_TIMEOUT) {
    priorityActive = false;
  }
}

void checkIRSensorEmergencyStop() {
  if (digitalRead(IR_SENSOR_BACK) == HIGH || digitalRead(IR_SENSOR_CLIFF) == LOW) {
    stopMotors();
    priorityActive = false;
  }
}

void executeAction(char action, uint8_t speed) {
  switch (action) {
    case 'F': forward(speed); break;
    case 'B': backward(speed); break;
    case 'L': turnLeft(speed); break;
    case 'R': turnRight(speed); break;
    case 'S': stopMotors(); priorityActive = false; break;
    default: stopMotors(); break;
  }
}

void sendAck(char action, uint8_t speed) {
  Serial.write(ACK_MARKER);
  Serial.write(action);
  Serial.write(speed);
}

// Motor control
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

void stopMotors() {
  analogWrite(MOTOR_A1, 0);
  analogWrite(MOTOR_A2, 0);
  analogWrite(MOTOR_B1, 0);
  analogWrite(MOTOR_B2, 0);
}
