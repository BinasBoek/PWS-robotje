#include <Servo.h>
#include <SoftwareSerial.h>

// Pin definitions for servos
#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 2

// HC-SR04 Sensor Pins
#define TRIG_PIN 8
#define ECHO_PIN 9

// Bluetooth Module pins
#define BT_TX_PIN 11
#define BT_RX_PIN 10

// Create instances for the servo motors
Servo servoLeft;
Servo servoRight;

// Create instance for SoftwareSerial (Bluetooth)
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// HC-SR04 sensor variables
long duration;
int distance;

// Command variables
char command;

void setup() {
  // Start serial communication (optional)
  Serial.begin(9600);
  btSerial.begin(9600);  // Start Bluetooth serial communication

  // Attach the servo motors to the corresponding pins
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  // Initialize the trigger and echo pins for HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set initial servo positions to forward
  servoLeft.writeMicroseconds(1511);  // Left servo moves forward
  servoRight.writeMicroseconds(1511); // Right servo moves forward

  Serial.println("Robot is ready to move.");
}

void loop() {
  // Check if Bluetooth data is available
  if (btSerial.available()) {
    command = btSerial.read(); // Read the incoming Bluetooth command
    Serial.print("Received command: ");
    Serial.println(command);

    // Handle Bluetooth command
    if (command == 'F') {
      moveForward();
    }
    else if (command == 'S') {
      stopMovement();
    }
    else if (command == 'B') {
      moveBackward();
    }
    else if (command == 'L') {
      turnLeft();
    }
    else if (command == 'R') {
      turnRight();
    }
    else {
      Serial.println("Unknown command");
    }
  }

  // Read the distance from the HC-SR04 sensor
  distance = getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  // If distance is less than 20 cm, stop the robot
  if (distance < 20) {
    stopMovement();
    Serial.println("Obstacle detected! Stopping.");
  }

  delay(100);  // Small delay for stability
}

// Function to get the distance from the HC-SR04 sensor
int getDistance() {
  // Send a 10ms pulse to trigger the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the pulse duration
  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (in cm)
  distance = duration * 0.0344 / 2;
  return distance;
}

// Function to move the robot forward
void moveForward() {
  servoLeft.write(800);  // Left servo moves forward
  servoRight.write(1600); // Right servo moves forward
  Serial.println("Moving Forward");
}

// Function to move the robot backward
void moveBackward() {
  servoLeft.writeMicroseconds(1600);  // Left servo moves backward
  servoRight.writeMicroseconds(800);   // Right servo moves backward
  Serial.println("Moving Backward");
}

// Function to stop the robot
void stopMovement() {
  servoLeft.writeMicroseconds(1511);   // Set left servo to neutral (stop)
  servoRight.writeMicroseconds(1511);  // Set right servo to neutral (stop)
  Serial.println("Stopped");
}

// Function to turn the robot left (stay in place)
void turnLeft() {
  servoLeft.writeMicroseconds(800);    // Left servo moves to 0 (stop or move backward)
  servoRight.writeMicroseconds(800);  // Right servo moves forward
  Serial.println("Turning Left");
}

// Function to turn the robot right (stay in place)
void turnRight() {
  servoLeft.writeMicroseconds(1600);   // Left servo moves forward
  servoRight.writeMicroseconds(1600); // Right servo moves to 180 (stop or move backward)
  Serial.println("Turning Right");
}

