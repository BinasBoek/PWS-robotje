#include <Servo.h>

// Define ultrasonic sensor pins
#define TRIG_PIN 11       // Trigger pin of HC-SR04
#define ECHO_PIN 12       // Echo pin of HC-SR04

// Define motor control pins
#define RIGHT_MOTOR_PIN 9 // Right motor
#define LEFT_MOTOR_PIN 10 // Left motor

// Define button pin
#define BUTTON_PIN 2      // Button pin voor on/off control

Servo rightMotor;
Servo leftMotor;

long duration;
int distance;
bool isRobotOn = false; // State van de robot
bool lastButtonState = HIGH; // Laatste state van de button
unsigned long lastDebounceTime = 0; // Laatste debounce time
unsigned long debounceDelay = 50; // Debounce delay

void setup() {
  // Initialiseer motoren
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);

  // Set up HC-SR04 sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set up button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Start met motors stopped
  stopMotors();
}

void loop() {
  // Read button state
  bool buttonState = digitalRead(BUTTON_PIN);

  // Check if button state changed 
  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If button state is LOW and it was HIGH eerder 
    if (buttonState == LOW && lastButtonState == HIGH) {
      isRobotOn = !isRobotOn; // Toggle robot state
    }
  }

  lastButtonState = buttonState;

  // If robot on, obstacle avoidance
  if (isRobotOn) {
    distance = measureDistance();

    if (distance < 10) {
      stopMotors();        // Stop robot
      delay(500);          // Pause

      turnRight90HalfSpeed(); // Turn 90 degrees right  half speed
      delay(500);             // Pause

      distance = measureDistance();
      if (distance < 10) {
        turnLeft90HalfSpeed(); // Turn 90 degrees left at half speed
        delay(500);            // Pause
      }

      turnLeft90HalfSpeed();   // terug naar original direction at half speed
      delay(500);              // Pause
      moveForwardHalfSpeed();  // ga door moving forward half speed
    } else {
      moveForwardHalfSpeed();  // Continue moving forward wanneer no obstacle
    }
  } else {
    stopMotors(); // Stop robot if turned off
  }
}

// Function move forward half speed
void moveForwardHalfSpeed() {
  rightMotor.write(135);  // Right motor  forward  half speed
  leftMotor.write(45);    // Left motor forward half speed
}

// Function stop motors
void stopMotors() {
  rightMotor.write(90);   // Stop rechter motor
  leftMotor.write(90);    // Stop linker motor
}

// Function 90 degrees right at half speed
void turnRight90HalfSpeed() {
  rightMotor.write(90);   // Stop rechter motor
  leftMotor.write(45);    // linker motor forward half speed
  delay(400);             //  90-degree turn
  stopMotors();
}

// Function  90 degrees  left half speed
void turnLeft90HalfSpeed() {
  rightMotor.write(135);  // rechter motor half speed
  leftMotor.write(90);    // Stop linker motor
  delay(400);             // 90-degree turn
  stopMotors();
}

// Function to measure the distance using the HC-SR04 sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // ultrasonic pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);

 
  distance = duration * 0.0344 / 2;

  return distance;
}

