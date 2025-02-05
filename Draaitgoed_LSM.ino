#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>


#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 2
#define TRIG_PIN 8
#define ECHO_PIN 9
#define BT_TX_PIN 11
#define BT_RX_PIN 10

Servo servoLeft;
Servo servoRight;

// Create instance for SoftwareSerial (Bluetooth)
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// HC-SR04 sensor variables
long duration;
int distance;

char command;

// LSM303 sensor instance (for accelerometer and magnetometer)
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);


float initialHeading;

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600); 

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize the LSM303 Magnetometer
  if (!mag.begin()) {
    Serial.println("Could not initialize the magnetometer.");
    while (1);  // Infinite loop if sensor initialization fails
  }

  // Set initial servo positions to neutral
  servoLeft.writeMicroseconds(1511); 
  servoRight.writeMicroseconds(1511);

  Serial.println("Robot is ready to move.");
}

void loop() {
  if (btSerial.available()) {
    command = btSerial.read();
    Serial.print("Received command: ");
    Serial.println(command);

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


  distance = getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);


  if (distance < 20) {
    stopMovement();
    Serial.println("Obstacle detected! Stopping.");
  }

  delay(100); 
}

int getDistance() {

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (in cm)
  distance = duration * 0.0344 / 2;
  return distance;
}

// Function to move the robot forward
void moveForward() {
  servoLeft.writeMicroseconds(800);
  servoRight.writeMicroseconds(1600);
  Serial.println("Moving Forward");

  
  delay(2000);
  stopMovement();
}

// Function to move the robot backward
void moveBackward() {
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(800); 
  Serial.println("Moving Backward");

  
  delay(2000); 
  stopMovement(); 
}

// Function to stop the robot
void stopMovement() {
  servoLeft.writeMicroseconds(1511);  
  servoRight.writeMicroseconds(1511); 
  Serial.println("Stopped");
}

// Function to turn the robot left (stay in place) with 90-degree change detection
void turnLeft() {
  initialHeading = getHeading(); 

  servoLeft.writeMicroseconds(800); 
  servoRight.writeMicroseconds(800); 
  Serial.println("Turning Left");

  // Keep turning until heading change reaches 90 degrees
  while (abs(getHeadingDifference(initialHeading, getHeading())) < 55) {
    delay(10); 
  }

  stopMovement(); 
  Serial.println("Turned Left 90 degrees");
}

// Function to turn the robot right (stay in place) with 90-degree change detection
void turnRight() {
  initialHeading = getHeading();  // Record the initial heading before turning

  servoLeft.writeMicroseconds(1600); 
  servoRight.writeMicroseconds(1600);
  Serial.println("Turning Right");

  // Keep turning until heading change reaches 90 degrees
  while (abs(getHeadingDifference(initialHeading, getHeading())) < 60) {
    delay(10); 
  }

  stopMovement(); 
  Serial.println("Turned Right 90 degrees");
}

// Function to get the current heading (azimuth) of the robot from the LSM303
float getHeading() {
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate the heading based on the magnetometer data
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading = heading * 180 / PI;

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

  return abs(diff);
}

