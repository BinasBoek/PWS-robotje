#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

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

// LSM303 sensor instance (for accelerometer and magnetometer)
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Variable to store the initial heading for the turns
float initialHeading;

void setup() {
  // Start serial communication
  Serial.begin(9600);
  btSerial.begin(9600);  // Start Bluetooth serial communication

  // Attach the servo motors to the corresponding pins
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  // Initialize the trigger and echo pins for HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the LSM303 Magnetometer
  if (!mag.begin()) {
    Serial.println("Could not initialize the magnetometer.");
    while (1);  // Infinite loop if sensor initialization fails
  }

  // Set initial servo positions to neutral
  servoLeft.writeMicroseconds(1511);  // Left servo in neutral (no movement)
  servoRight.writeMicroseconds(1511); // Right servo in neutral (no movement)

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
  servoLeft.writeMicroseconds(800);  // Left servo moves forward
  servoRight.writeMicroseconds(1600); // Right servo moves forward
  Serial.println("Moving Forward");

  // Wait for 2 seconds and stop the movement
  delay(2000); // Move forward for 2 seconds
  stopMovement(); // Stop after 2 seconds
}

// Function to move the robot backward
void moveBackward() {
  servoLeft.writeMicroseconds(1600);  // Left servo moves backward
  servoRight.writeMicroseconds(800);   // Right servo moves backward
  Serial.println("Moving Backward");

  // Wait for 2 seconds and stop the movement
  delay(2000); // Move backward for 2 seconds
  stopMovement(); // Stop after 2 seconds
}

// Function to stop the robot
void stopMovement() {
  servoLeft.writeMicroseconds(1511);   // Set left servo to neutral (stop)
  servoRight.writeMicroseconds(1511);  // Set right servo to neutral (stop)
  Serial.println("Stopped");
}

// Function to turn the robot left (stay in place) with 90-degree change detection
void turnLeft() {
  initialHeading = getHeading();  // Record the initial heading before turning

  servoLeft.writeMicroseconds(800);    // Left servo moves backward
  servoRight.writeMicroseconds(800);   // Right servo moves forward
  Serial.println("Turning Left");

  // Keep turning until heading change reaches 90 degrees
  while (abs(getHeadingDifference(initialHeading, getHeading())) < 55) {
    delay(10);  // Allow time for turning
  }

  stopMovement();  // Stop after completing the 90-degree turn
  Serial.println("Turned Left 90 degrees");
}

// Function to turn the robot right (stay in place) with 90-degree change detection
void turnRight() {
  initialHeading = getHeading();  // Record the initial heading before turning

  servoLeft.writeMicroseconds(1600);   // Left servo moves forward
  servoRight.writeMicroseconds(1600);  // Right servo moves backward
  Serial.println("Turning Right");

  // Keep turning until heading change reaches 90 degrees
  while (abs(getHeadingDifference(initialHeading, getHeading())) < 60) {
    delay(10);  // Allow time for turning
  }

  stopMovement();  // Stop after completing the 90-degree turn
  Serial.println("Turned Right 90 degrees");
}

// Function to get the current heading (azimuth) of the robot from the LSM303
float getHeading() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate the heading based on the magnetometer data
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading = heading * 180 / PI;  // Convert from radians to degrees

  // Normalize the heading to be in the range 0 to 360
  if (heading < 0) {
    heading += 360;
  }

  return heading;
}

// Function to calculate the difference in heading considering wraparound
float getHeadingDifference(float initialHeading, float currentHeading) {
  float diff = currentHeading - initialHeading;

  // Normalize the difference to be in the range -180 to 180 degrees
  if (diff > 90) {
    diff -= 360;
  } else if (diff < -90) {
    diff += 360;
  }

  return abs(diff);  // Return the absolute value of the difference
}

