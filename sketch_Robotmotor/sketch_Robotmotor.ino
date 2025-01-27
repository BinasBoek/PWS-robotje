#include <Servo.h>

// Define ultrasonic sensor pins
#define TRIG_PIN 11       // Trigger pin of the HC-SR04
#define ECHO_PIN 12       // Echo pin of the HC-SR04

// Define motor control pins
#define RIGHT_MOTOR_PIN 9 // Right motor (servo)
#define LEFT_MOTOR_PIN 10 // Left motor (servo)

Servo rightMotor;
Servo leftMotor;

long duration;
int distance;

void setup() {
  // Initialize motors
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);
  
  // Set up the HC-SR04 sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Start the robot moving forward
  moveForward();
}

void loop() {
  // Measure the distance using the ultrasonic sensor
  distance = measureDistance();
  
  // If an object is detected closer than 10 cm
  if (distance < 10) {
    stopMotors();         // Stop the robot
    delay(500);           // Pause briefly
    moveRight();          // Turn right for 0.8 seconds
    delay(800);
    moveLeft();           // Turn left for 0.8 seconds
    delay(800);
    moveForward();        // Resume moving forward
  } else {
    // Continue moving forward if no obstacle is detected
    moveForward();
  }
}

// Function to move forward (reversed directions)
void moveForward() {
  rightMotor.write(180);  // Right motor moves forward
  leftMotor.write(0);     // Left motor moves forward
}

// Function to stop motors
void stopMotors() {
  rightMotor.write(90);   // Stop right motor
  leftMotor.write(90);    // Stop left motor
}

// Function to turn right
void moveRight() {
  rightMotor.write(0);    // Right motor moves backward
  leftMotor.write(0);     // Left motor moves forward
}

// Function to turn left
void moveLeft() {
  rightMotor.write(180);  // Right motor moves forward
  leftMotor.write(180);   // Left motor moves backward
}

// Function to measure the distance using the HC-SR04 sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Trigger the ultrasonic pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure the duration of the echo
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance (distance = duration * speed of sound / 2)
  // The speed of sound is approximately 343 meters per second at room temperature
  distance = duration * 0.0344 / 2;
  
  return distance;
}
