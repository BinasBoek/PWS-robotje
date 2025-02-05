#include <Servo.h>


#define TRIG_PIN 11
#define ECHO_PIN 12
#define RIGHT_MOTOR_PIN 9 
#define LEFT_MOTOR_PIN 10 

Servo rightMotor;
Servo leftMotor;

long duration;
int distance;

const int buttonPin1 = 2;

// Variables
int buttonState = 0;
int lastButtonState = 0;
bool motorsOn = false;
int action = 0;

void setup() {
  rightMotor.attach(RIGHT_MOTOR_PIN);
  leftMotor.attach(LEFT_MOTOR_PIN);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(buttonPin1, INPUT);

  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);

  stopMotors();
}

void loop() {
  buttonState = digitalRead(buttonPin1);
  if (buttonState == HIGH && lastButtonState == LOW) {
    motorsOn = !motorsOn;
    if (motorsOn) {
      moveForwardHalfSpeed();
    } else {
      stopMotors();
    }
    delay(50);
  }

  lastButtonState = buttonState;


  if (motorsOn) {
    digitalWrite(5, HIGH);
  } else {
    digitalWrite(5, LOW);
  }

  digitalWrite(3, action);
  delay(25);

  distance = measureDistance();

  if (motorsOn && distance < 10) {
    stopMotors();
    delay(500);
    
    turnRight90HalfSpeed();
    delay(500);


    distance = measureDistance();
    if (distance < 10) {
      turnLeft90HalfSpeed(); 
      delay(500);
    }
    
    turnLeft90HalfSpeed();
    delay(500);
    moveForwardHalfSpeed(); 
  }
}

void moveForwardHalfSpeed() {
  rightMotor.write(135);
  leftMotor.write(45);
}

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
  
  // Calculate distance in cm
  distance = duration * 0.0344 / 2;
  
  return distance;
}
