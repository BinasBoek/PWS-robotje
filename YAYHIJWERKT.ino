#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>


#define TRIG_PIN 8
#define ECHO_PIN 9
#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 2

// Bluetooth Module pins
#define BT_TX_PIN 10
#define BT_RX_PIN 11

// Create instances  servo motors and Bluetooth serial
Servo servoLeft;
Servo servoRight;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); 

// Create instance for   LSM303DLHC sensor (accelerometer and compass)
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);

void setup() {
  // Start serial communication
  Serial.begin(9600);
  btSerial.begin(9600);  // Bluetooth baud rate

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  // Set   pin modes for   ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize LSM303DLHC
  if (!accel.begin()) {
    Serial.println("Could not initialize   accelerometer.");
    while (1);
  }

  Serial.println("Robot is ready!");

  // Initial positions for   servos (optional)
  servoLeft.write(90); 
  servoRight.write(90);
}

void loop() {
  if (btSerial.available()) {
    char incomingByte = btSerial.read(); // Read Bluetooth command
    Serial.print("Received Bluetooth command: ");
    Serial.println(incomingByte);

    // Control based on Bluetooth command
    if (incomingByte == 'F') {
      // Move forward
      servoLeft.write(90);   
      servoRight.write(90);  
      Serial.println("Moving Forward");
    }
    else if (incomingByte == 'B') {
      // Move backward
      servoLeft.write(180);
      servoRight.write(0);
      Serial.println("Moving Backward");
    }
    else if (incomingByte == 'L') {
      // Turn left
      servoLeft.write(0);
      servoRight.write(90); 
      Serial.println("Turning Left");
    }
    else if (incomingByte == 'R') {
      // Turn right
      servoLeft.write(90);
      servoRight.write(180);
      Serial.println("Turning Right");
    }
    else if (incomingByte == 'S') {
      // Stop 
      servoLeft.writeMicroseconds(1511); 
      servoRight.writeMicroseconds(1511);
      Serial.println("Stopping");
    }
    else {
      Serial.println("Unknown command received");
    }
  }

  delay(100); 
}
