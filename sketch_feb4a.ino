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

// Create instances voor servo motoren
Servo servoLeft;
Servo servoRight;

// Create instance voor SoftwareSerial (Bluetooth)
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// HC-SR04 sensor variables
long duration;
int distance;

// LSM303 Accelerometer and Magnetometer
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(54321);

// Command variables
char command;
float initialHeading;

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600); // Start Bluetooth communication

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  // Initialize Ultrasonic Sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize LSM303
  if (!accel.begin()) {
    Serial.println("Accelerometer not found!");
    while (1);
  }
  if (!mag.begin()) {
    Serial.println("Magnetometer not found!");
    while (1);
  }

// servos stil bij startup
  stopMovement();
  Serial.println("Robot ready!");
}

void loop() {
  // Check voor Bluetooth input
  if (btSerial.available()) {
    command = btSerial.read();
    Serial.print("Received command: ");
    Serial.println(command);

    // If obstacle detected, geen movement
    if (getDistance() < 20) {
      stopMovement();
      Serial.println("Obstacle detected! Stopping.");
      return;
    }

    // Execute movement command
    executeCommand(command);
  }

  delay(100);
}

// Function om execute movement gebaseerd op Bluetooth command
void executeCommand(char cmd) {
  if (cmd == 'F') moveForward();
  else if (cmd == 'B') moveBackward();
  else if (cmd == 'L') turnLeft();
  else if (cmd == 'R') turnRight();
  else if (cmd == 'S') stopMovement();
  else Serial.println("Unknown command");
}

// Function get distance van HC-SR04 sensor
int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.0344 / 2; // Convert tijd naar distance in cm
  Serial.print("Distance: ");
  Serial.println(distance);
  return distance;
}

// Function move forward voor 2 secondes
void moveForward() {
  servoLeft.writeMicroseconds(800);
  servoRight.writeMicroseconds(1600);
  Serial.println("Moving Forward");
  delay(2000);
  stopMovement();
}

// Function move backward voor 2 secondes
void moveBackward() {
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(800);
  Serial.println("Moving Backward");
  delay(2000);
  stopMovement();
}

// full stop
void stopMovement() {
  servoLeft.writeMicroseconds(1511);
  servoRight.writeMicroseconds(1511);
  delay(200); 
  Serial.println("Stopped");
}

// Function turn left met 90-degree detection
void turnLeft() {
  initialHeading = getHeading(); 

  servoLeft.writeMicroseconds(800);
  servoRight.writeMicroseconds(800);
  Serial.println("Turning Left");

  // Keep turning tot heading change reaches 55 degrees
  while (abs(getHeadingDifference(initialHeading, getHeading())) < 55) {
    delay(10);
  }

  stopMovement();  // Stop na 90-degree turn
  Serial.println("Turned Left 90 degrees");
}

//turn right  90-degree detectie
void turnRight() {
  initialHeading = getHeading();  // initial heading voor turning

  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1600);
  Serial.println("Turning Right");

  //  turning totdat heading change reaches 60 degrees
  while (abs(getHeadingDifference(initialHeading, getHeading())) < 60) {
    delay(10);
  }

  stopMovement();  // Stop na 90-degree turn
  Serial.println("Turned Right 90 degrees");
}

// Function voor robot's current heading
float getHeading() {
  sensors_event_t event;
  mag.getEvent(&event);
  
  float heading = atan2(event.magnetic.y, event.magnetic.x) * 180 / PI;
  if (heading < 0) heading += 360;
  
  return heading;
}

// Function calculate heading change
float getHeadingDifference(float start, float current) {
  float diff = current - start;
  if (diff < -180) diff += 360;
  if (diff > 180) diff -= 360;
  return abs(diff);
}
