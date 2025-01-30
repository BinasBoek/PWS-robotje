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
  if (btSerial.available()) {
    char incomingByte = btSerial.read(); // Read the incoming Bluetooth command
    Serial.print("Received Bluetooth command: ");
    Serial.println(incomingByte);

    // Control based on Bluetooth command
    if (incomingByte == 'F') {
      // Move forward
      servoLeft.write(90);   // Set to neutral position (move forward)
      servoRight.write(90);  // Set to neutral position (move forward)
      Serial.println("Moving Forward");
    }
    else if (incomingByte == 'B') {
      // Move backward
      servoLeft.write(180);  // Move in reverse
      servoRight.write(0);   // Move in reverse
      Serial.println("Moving Backward");
    }
    else if (incomingByte == 'L') {
      // Turn left
      servoLeft.write(0);    // Turn left (left servo goes to 0)
      servoRight.write(90);  // Right servo moves forward
      Serial.println("Turning Left");
    }
    else if (incomingByte == 'R') {
      // Turn right
      servoLeft.write(90);   // Left servo moves forward
      servoRight.write(180); // Right servo moves in reverse (turning right)
      Serial.println("Turning Right");
    }
    else if (incomingByte == 'S') {
      // Stop - Set both servos to neutral (stop moving)
      servoLeft.writeMicroseconds(1500);  // Set left servo to neutral
      servoRight.writeMicroseconds(1500); // Set right servo to neutral
      Serial.println("Stopping");
    }
    else {
      Serial.println("Unknown command received");
    }
  }

  delay(100);  // Small delay for stability
}
