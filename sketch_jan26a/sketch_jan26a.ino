#include <Servo.h>

Servo myServo1;    // Create a servo object for the first motor
Servo myServo2;    // Create a servo object for the second motor
const int buttonPin = 3;  // Button connected to pin 3
const int servoPin1 = 2;  // First servo connected to pin 2
const int servoPin2 = 4;  // Second servo connected to pin 4
const int trigPin = 9;    // HC-SR04 Trigger pin connected to pin 9
const int echoPin = 10;   // HC-SR04 Echo pin connected to pin 10

int buttonState = 0;  // Variable to hold the button state
int lastButtonState = 0;  // Variable to store last button state
bool motorState = false;  // Track if the motors are on or off

// Distance threshold for obstacle detection (in centimeters)
const int obstacleThreshold = 20;

void setup() {
  Serial.begin(9600);  // Initialize serial communication at 9600 baud rate
  myServo1.attach(servoPin1);  // Attach the first servo to pin 2
  myServo2.attach(servoPin2);  // Attach the second servo to pin 4
  pinMode(buttonPin, INPUT_PULLUP);  // Set button pin as input with internal pull-up resistor
  pinMode(trigPin, OUTPUT);  // Set the trig pin as an output
  pinMode(echoPin, INPUT);   // Set the echo pin as an input

  myServo1.write(0);  // Initialize first servo to the 'off' position
  myServo2.write(0);  // Initialize second servo to the 'off' position
}

void loop() {
  buttonState = digitalRead(buttonPin);  // Read the button state

  // Check if the button has been pressed (low state due to pull-up)
  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50);  // Debounce delay
    
    if (motorState == false) {
      motorState = true;
      myServo1.attach(servoPin1);  // Reattach the first servo
      myServo2.attach(servoPin2);  // Reattach the second servo
      myServo1.write(90);  // Move the first servo to the 'on' position
      myServo2.write(90);  // Move the second servo to the 'on' position
      Serial.println("Motors ON");  // Report that motors are on
    } else {
      motorState = false;
      myServo1.write(0);  // Move the first servo to the 'off' position
      myServo2.write(0);  // Move the second servo to the 'off' position
      myServo1.detach();  // Detach the first servo
      myServo2.detach();  // Detach the second servo
      Serial.println("Motors OFF");  // Report that motors are off
    }
  }

  // Obstacle detection logic
  long distance = measureDistance();  // Measure the distance from the sensor
  if (distance < obstacleThreshold && motorState == true) {  // If an obstacle is detected while motors are on
    Serial.println("Obstacle detected! Turning robot.");
    turnRobot();  // Make the robot turn 90 degrees
  }

  // Update the last button state for the next loop
  lastButtonState = buttonState;
}

// Function to measure distance using the HC-SR04 sensor
long measureDistance() {
  digitalWrite(trigPin, LOW);  // Ensure the trig pin is low
  delayMicroseconds(2);  // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH);  // Trigger the ultrasonic pulse
  delayMicroseconds(10);  // Wait for 10 microseconds
  digitalWrite(trigPin, LOW);  // Stop triggering the pulse

  // Read the echo pulse
  long duration = pulseIn(echoPin, HIGH);  // Measure the duration of the pulse
  long distance = duration * 0.034 / 2;  // Calculate the distance in cm

  return distance;  // Return the calculated distance
}

// Function to make the robot turn 90 degrees
void turnRobot() {
  // Rotate: First servo forward and second servo backward
  myServo1.write(120);  // Adjust to move forward
  myServo2.write(60);   // Adjust to move backward
  delay(500);  // Adjust delay for a 90-degree turn (depends on motor speed and robot configuration)

  // Stop the robot after the turn
  myServo1.write(90);  // Neutral position for the first servo
  myServo2.write(90);  // Neutral position for the second servo
  Serial.println("Turn complete.");
}

