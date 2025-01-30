#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// Pin definitions
#define TRIG_PIN 8
#define ECHO_PIN 9
#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 2

// Bluetooth Module pins
#define BT_TX_PIN 10
#define BT_RX_PIN 11

// Create instances for the servo motors and Bluetooth serial
Servo servoLeft;
Servo servoRight;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // Bluetooth module on RX/TX

// Create instance for the LSM303DLHC sensor (accelerometer and compass)
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);

void setup() {
  // Start serial communication
  Serial.begin(9600);
  btSerial.begin(9600);  // Bluetooth baud rate

  // Attach the servo motors
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  // Set the pin modes for the ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize LSM303DLHC
  if (!accel.begin()) {
    Serial.println("Could not initialize the accelerometer.");
    while (1);
  }

  Serial.println("Robot is ready!");

  // Initial positions for the servos (optional)
  servoLeft.write(90);  // Set left servo to forward position
  servoRight.write(90); // Set right servo to forward position
}

void loop() {
  // Measure distance using HC-SR04 ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1;  // Calculate distance in cm

  // Print distance for debugging
  Serial.print("Distance: ");
  Serial.println(distance);

  // Move the robot based on distance measurement
  if (distance > 10) { // No obstacle detected
    // Move forward
    servoLeft.write(90);  // Move left servo forward
    servoRight.write(90); // Move right servo forward
  } else { // Obstacle detected
    // Stop and turn (basic avoidance strategy)
    servoLeft.write(0);   // Turn left
    servoRight.write(180); // Turn right
  }

  // Read accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  // Output accelerometer readings for debugging
  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(" Z: ");
  Serial.println(event.acceleration.z);

  // Check for Bluetooth input to control the robot
  if (btSerial.available()) {
    char incomingByte = btSerial.read(); // Read the incoming Bluetooth command
    Serial.print("Received Bluetooth command: ");
    Serial.println(incomingByte);

    // Example control using Bluetooth
    if (incomingByte == 'F') {
      // Move forward
      servoLeft.write(90);
      servoRight.write(90);
    }
    if (incomingByte == 'B') {
      // Move backward
      servoLeft.write(180);
      servoRight.write(0);
    }
    if (incomingByte == 'L') {
      // Turn left
      servoLeft.write(0);
      servoRight.write(90);
    }
    if (incomingByte == 'R') {
      // Turn right
      servoLeft.write(90);
      servoRight.write(180);
    }
  }

  delay(100);  // Small delay for stability
}
