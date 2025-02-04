#include <Servo.h>
#include <SoftwareSerial.h>

// Pin definitions
#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 2
#define TRIG_PIN 8
#define ECHO_PIN 9
#define BT_TX_PIN 11
#define BT_RX_PIN 10

Servo servoLeft;
Servo servoRight;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

long duration;
int distance;
bool movementStarted = false;
int originalTurnDirection = 0;  // 1 = Right, -1 = Left

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  stopMovement();
  Serial.println("Robot Ready. Send 'S' to Start, 'X' to Stop, or 'R' to Turn 180°.");
}

void loop() {
  checkBluetoothCommand(); // ✅ Always check for new commands

  if (movementStarted) {
    obstacleAvoidance();
  }
}

// ✅ Check Bluetooth for 'S' (Start), 'X' (Stop), and 'R' (Turn 180°)
void checkBluetoothCommand() {
  if (btSerial.available()) {
    char command = btSerial.read();
    Serial.print("Received command: ");
    Serial.println(command);

    if (command == 'S' && !movementStarted) {
      movementStarted = true;
      Serial.println("Starting movement...");
    } 
    else if (command == 'X') {  // ✅ Stop command
      movementStarted = false;
      stopMovement();
      Serial.println("Robot Stopped!");
    } 
    else if (command == 'R') {  // ✅ Turn 180° command
      turn180();
    }
  }
}

// ✅ Get Distance Measurement
int getDistance() {
  delay(50); // Short delay for accuracy

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.0344 / 2; // Convert to cm

  if (distance == 0 || distance > 200) { // Ignore invalid readings
    distance = 200;
  }

  Serial.print("Distance Measured: ");
  Serial.println(distance);
  return distance;
}

// ✅ Movement Functions
void moveForward(int time) {
  servoLeft.writeMicroseconds(1200);
  servoRight.writeMicroseconds(1800);
  Serial.println("Moving Forward");
  delay(time);
  stopMovement();
}

void moveBackward(int time) {
  servoLeft.writeMicroseconds(1800);
  servoRight.writeMicroseconds(1200);
  Serial.println("Moving Backward");
  delay(time);
  stopMovement();
}

void turnRight(int time) {
  Serial.println("Turning Right (Wide Turn)");
  servoLeft.writeMicroseconds(1800);
  servoRight.writeMicroseconds(1800);
  delay(time);
  stopMovement();
}

void turnLeft(int time) {
  Serial.println("Turning Left (Wide Turn)");
  servoLeft.writeMicroseconds(1200);
  servoRight.writeMicroseconds(1200);
  delay(time);
  stopMovement();
}

void turn180() {
  // Turn the robot 180 degrees
  Serial.println("Turning 180 degrees");
  turnRight(2833);  // Use 2833 to turn 180 degrees
  Serial.println("Turned 180 degrees");
}

// ✅ Stop Movement
void stopMovement() {
  servoLeft.writeMicroseconds(1511);
  servoRight.writeMicroseconds(1511);
  Serial.println("Stopped");
  delay(500);
}

// ✅ Improved Obstacle Avoidance (Now Checks Bluetooth Command)
void obstacleAvoidance() {
  while (movementStarted) {
    checkBluetoothCommand();  // ✅ Check Bluetooth every cycle

    moveForward(600); 
    int frontDistance = getDistance();

    if (frontDistance < 30) {  
      Serial.println("🚨 Obstacle Detected! Deciding turn direction...");
      stopMovement();

      moveBackward(100);

      Serial.println("🔄 Scanning Left...");
      turnLeft(1416.5);
      int leftDistance = getDistance();

      Serial.println("🔄 Scanning Right...");
      turnRight(2833);
      int rightDistance = getDistance();

      turnLeft(1416.5);

      if (rightDistance > leftDistance) {
        Serial.println("✅ Turning Right (More Space)");
        turnRight(1416.5);
        originalTurnDirection = 1;
      } else {
        Serial.println("✅ Turning Left (More Space)");
        turnLeft(1416.5);
        originalTurnDirection = -1;
      }

      moveForward(2500);

      if (originalTurnDirection == 1) {
        Serial.println("🔄 Returning to Original Direction (Turning Left)");
        turnLeft(1416.5);
      } else if (originalTurnDirection == -1) {
        Serial.println("🔄 Returning to Original Direction (Turning Right)");
        turnRight(1416.5);
      }
    }
  }
}


