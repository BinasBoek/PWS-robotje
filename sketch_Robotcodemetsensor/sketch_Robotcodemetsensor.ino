#include <Servo.h>

// ultrasonic sensor pins
#define TRIG_PIN 11       // Trigger pin HC-SR04
#define ECHO_PIN 12       // Echo pin HC-SR04

// motor control pins
#define RIGHT_MOTOR_PIN 9 
#define LEFT_MOTOR_PIN 10 

Servo rightMotor;
Servo leftMotor;

long duration;
int distance;

void setup() {
  // initaliseer motoren
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);
  
  // Set up HC-SR04 sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // moving half speed
  moveForwardHalfSpeed();
}

void loop() {
  // Measure distance using ultrasonic sensor
  distance = measureDistance();
  
  // if obstacle detected within 10 cm
  if (distance < 10) {
    stopMotors();        // Stop robot
    delay(500);          
    
    turnRight90HalfSpeed(); // turn 90 degrees right half speed
    delay(500);             

    // Check for obstacle
    distance = measureDistance();
    if (distance < 10) {
      turnLeft90HalfSpeed(); 
      delay(500);            
    }
    
    turnLeft90HalfSpeed();   
    delay(500);              
    moveForwardHalfSpeed();  
  } else {
    
    moveForwardHalfSpeed();
  }
}


void moveForwardHalfSpeed() {
  rightMotor.write(135);  
  leftMotor.write(45);    
}

stop motors
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
