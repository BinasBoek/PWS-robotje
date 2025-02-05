#include <Servo.h>

// Define ultrasonic sensor pins
#define TRIG_PIN 11       // Trigger pin HC-SR04
#define ECHO_PIN 12       // Echo pin HC-SR04

// Define motor control pins
#define RIGHT_MOTOR_PIN 9 // Right motor
#define LEFT_MOTOR_PIN 10 // Left motor

// Define button pin
#define TOGGLE_BUTTON_PIN 2 // Start/Stop button pin

Servo rightMotor;
Servo leftMotor;

long duration;
int distance;
bool isRobotRunning = false; 
bool lastButtonState = HIGH; // knop

void setup() {
  // Initialiseer motoren
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);
  
  // Set HC-SR04 sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Set toggle button
  pinMode(TOGGLE_BUTTON_PIN, INPUT_PULLUP); // Use internal pull-up resistor
  
  // Stop motors 
  stopMotors();
}

void loop() {
  // Check the state of the toggle button
  bool buttonState = digitalRead(TOGGLE_BUTTON_PIN);
  
  // button state changes from HIGH to LOW (button pressed)
  if (buttonState == LOW && lastButtonState == HIGH) {
    isRobotRunning = !isRobotRunning; // Toggle robot running state
    delay(10000); // Debounce delay
  }
  lastButtonState = buttonState; // update last button state
  
  if (isRobotRunning) {
    obstacleAvoidance();
  } else {
    stopMotors(); // stopmotors
  }
}

void obstacleAvoidance() {
  distance = measureDistance();
  
  // If obstacle is detected within 10 cm
  if (distance < 10) {
    stopMotors();            // Stop robot
    delay(500);              // Pause
    
    turnRight90HalfSpeed();  // 90 degrees to the right
    delay(500);              // Pause 

    // Check if obstacle
    distance = measureDistance();
    if (distance < 10) {
      turnLeft90HalfSpeed(); // 90 degrees to the left
      delay(500);            // Pause
    }
    
    turnLeft90HalfSpeed();   // Turn back to original direction
    delay(500);              // Pause
    moveForwardHalfSpeed();  // Resume moving forward 
  } else {
    // moving forward if no obstacle
    moveForwardHalfSpeed();
  }
}

// move forward at half speed
void moveForwardHalfSpeed() {
  rightMotor.write(135);  // Right motor 
  leftMotor.write(45);    // Left motor 
}

// stop motors
void stopMotors() {
  rightMotor.write(90);   
  leftMotor.write(90);    
}


void turnRight90HalfSpeed() {
  rightMotor.write(90);   
  leftMotor.write(45);    
  delay(400);             
  stopMotors();
}


void turnLeft90HalfSpeed() {
  rightMotor.write(135); 
  leftMotor.write(90);    
  delay(400);             
  stopMotors();
}

long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
 
  duration = pulseIn(ECHO_PIN, HIGH);
  
 
  distance = duration * 0.0344 / 2;
  
  return distance;
}
