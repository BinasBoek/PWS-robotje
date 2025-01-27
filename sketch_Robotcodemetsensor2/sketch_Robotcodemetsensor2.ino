#include <Servo.h>

// Define ultrasonic sensor pins
#define TRIG_PIN 11       // Trigger pin of the HC-SR04
#define ECHO_PIN 12       // Echo pin of the HC-SR04

// Define motor control pins
#define RIGHT_MOTOR_PIN 9 // Right motor (servo)
#define LEFT_MOTOR_PIN 10 // Left motor (servo)

// Define button pin
#define TOGGLE_BUTTON_PIN 2 // Start/Stop button pin

Servo rightMotor;
Servo leftMotor;

long duration;
int distance;
bool isRobotRunning = false; // Tracks if the robot is running
bool lastButtonState = HIGH; // Tracks the previous state of the button (HIGH = unpressed)

void setup() {
  // Initialize motors
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);
  
  // Set up the HC-SR04 sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Set up the toggle button
  pinMode(TOGGLE_BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up resistor
  
  // Stop motors initially
  stopMotors();
}

void loop() {
  // Check the state of the toggle button
  bool buttonState = digitalRead(TOGGLE_BUTTON_PIN);
  
  // If the button state changes from HIGH to LOW (button pressed)
  if (buttonState == LOW && lastButtonState == HIGH) {
    isRobotRunning = !isRobotRunning; // Toggle the robot running state
    delay(10000); // Debounce delay
  }
  lastButtonState = buttonState; // Update the last button state
  
  // If the robot is running, execute the obstacle avoidance behavior
  if (isRobotRunning) {
    obstacleAvoidance();
  } else {
    stopMotors(); // If not running, stop the motors
  }
}

// Obstacle avoidance behavior
void obstacleAvoidance() {
  // Measure the distance using the ultrasonic sensor
  distance = measureDistance();
  
  // If an obstacle is detected within 10 cm
  if (distance < 10) {
    stopMotors();            // Stop the robot
    delay(500);              // Brief pause before turning
    
    turnRight90HalfSpeed();  // Turn 90 degrees to the right at half speed
    delay(500);              // Pause after the turn

    // Check if there's another obstacle
    distance = measureDistance();
    if (distance < 10) {
      turnLeft90HalfSpeed(); // Turn 90 degrees to the left at half speed to avoid
      delay(500);            // Pause
    }
    
    turnLeft90HalfSpeed();   // Turn back to the original direction at half speed
    delay(500);              // Pause
    moveForwardHalfSpeed();  // Resume moving forward at half speed
  } else {
    // Continue moving forward if no obstacle is detected
    moveForwardHalfSpeed();
  }
}

// Function to move forward at half speed
void moveForwardHalfSpeed() {
  rightMotor.write(135);  // Right motor moves forward at half speed
  leftMotor.write(45);    // Left motor moves forward at half speed
}

// Function to stop motors
void stopMotors() {
  rightMotor.write(90);   // Stop right motor
  leftMotor.write(90);    // Stop left motor
}

// Function to turn 90 degrees to the right at half speed
void turnRight90HalfSpeed() {
  rightMotor.write(90);   // Stop right motor
  leftMotor.write(45);    // Left motor moves forward at half speed
  delay(400);             // Adjust timing for a precise 90-degree turn
  stopMotors();
}

// Function to turn 90 degrees to the left at half speed
void turnLeft90HalfSpeed() {
  rightMotor.write(135);  // Right motor moves forward at half speed
  leftMotor.write(90);    // Stop left motor
  delay(400);             // Adjust timing for a precise 90-degree turn
  stopMotors();
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
