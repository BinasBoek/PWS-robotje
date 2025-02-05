#include <Servo.h>

Servo myServo1;    // servo object eerste motor
Servo myServo2;    // servo object voor de tweede motor
const int buttonPin = 3;  // knop naar pin 3
const int servoPin1 = 2;  // servo pin 2
const int servoPin2 = 4;  // servo pin 4
const int trigPin = 9;    // HC-SR04 Trigger pin pin 9
const int echoPin = 10;   // HC-SR04 Echo pin pin 10

int buttonState = 0;  
int lastButtonState = 0;  
bool motorState = false;  

// Distance threshold obstacle detection
const int obstacleThreshold = 20;

void setup() {
  Serial.begin(9600);  // serial communication 9600 baud rate
  myServo1.attach(servoPin1);  // Attach servo pin 2
  myServo2.attach(servoPin2);  // Attach servo pin 4
  pinMode(buttonPin, INPUT_PULLUP);  // Set button pin als input met internal pull-up resistor
  pinMode(trigPin, OUTPUT);  // Set trig pin as output
  pinMode(echoPin, INPUT);   // Set echo pin as input

  myServo1.write(0);  // servo off
  myServo2.write(0);  // servo off
}

void loop() {
  buttonState = digitalRead(buttonPin);  // Read button state

  if (buttonState == LOW && lastButtonState == HIGH) {
    delay(50);  // Debounce delay
    
    if (motorState == false) {
      motorState = true;
      myServo1.attach(servoPin1);  // Reattach first servo
      myServo2.attach(servoPin2);  // Reattach second servo
      myServo1.write(90);  // Move first servo to 'on' position
      myServo2.write(90);  // Move second servo to 'on' position
      Serial.println("Motors ON");  // Report motors are on
    } else {
      motorState = false;
      myServo1.write(0);  // Move  first servo to  'off' position
      myServo2.write(0);  // Move  second servo to  'off' position
      myServo1.detach();  // Detach  first servo
      myServo2.detach();  // Detach  second servo
      Serial.println("Motors OFF");  // Report  motors  off
    }
  }

  // Obstacle detection 
  long distance = measureDistance();  // Measure  distance from the sensor
  if (distance < obstacleThreshold && motorState == true) {  // If an obstacle is detected terwijl motors on
    Serial.println("Obstacle detected! Turning robot.");
    turnRobot();  // robot turn 90 degrees
  }

  // Update button state voor next loop
  lastButtonState = buttonState;
}

// Function measure distance 
long measureDistance() {
  digitalWrite(trigPin, LOW);  // Ensure the trig pin is low
  delayMicroseconds(2);  // Wait for 2 microseconds
  digitalWrite(trigPin, HIGH);  // Trigger the ultrasonic pulse
  delayMicroseconds(10);  // Wait for 10 microseconds
  digitalWrite(trigPin, LOW);  // Stop triggering the pulse

  // Read echo pulse
  long duration = pulseIn(echoPin, HIGH);  // Measure duration of the pulse
  long distance = duration * 0.034 / 2;  // Calculate distance in cm

  return distance;  // Return calculated distance
}

// Function to make the robot turn 90 degrees
void turnRobot() {
  // Rotate: First servo forward and second servo backward
  myServo1.write(120);  // forward
  myServo2.write(60);   // backward
  delay(500);  

  // Stop na turn
  myServo1.write(90);  // Neutral position
  myServo2.write(90);  // Neutral position 
  Serial.println("Turn complete.");
}

