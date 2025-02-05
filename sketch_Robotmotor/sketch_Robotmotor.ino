#include <Servo.h>

// US pins
#define TRIG_PIN 11       
#define ECHO_PIN 12       

//  motor pins
#define RIGHT_MOTOR_PIN 9 
#define LEFT_MOTOR_PIN 10 

Servo rightMotor;
Servo leftMotor;

long duration;
int distance;

void setup() {
  // motors
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);
  
  // Set up HC-SR04 sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Start robot moving forward
  moveForward();
}

void loop() {
  distance = measureDistance();
  
  if (distance < 10) {
    stopMotors();         // Stop robot
    delay(500);           // Pause
    moveRight();          // rechts
    delay(800);
    moveLeft();           // link
    delay(800);
    moveForward();        // moving forward
  } else {
    // forward als er geen obstakel is
    moveForward();
  }
}

// Function move forward 
void moveForward() {
  rightMotor.write(180);  
  leftMotor.write(0);     
}

// Function stop motors
void stopMotors() {
  rightMotor.write(90);   // Stop
  leftMotor.write(90);    // Stop 
}

// Function right
void moveRight() {
  rightMotor.write(0);    // backw
  leftMotor.write(0);     // forw
}

// Function left
void moveLeft() {
  rightMotor.write(180);  // forw
  leftMotor.write(180);   // backw
}

//  measure distance HC-SR04 sensor
long measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Trigger ultrasonic pulse
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Measure duration echo
  duration = pulseIn(ECHO_PIN, HIGH);
  
  
  distance = duration * 0.0344 / 2;
  
  return distance;
}
