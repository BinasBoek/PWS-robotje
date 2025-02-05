#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#define TRIG_PIN 8
#define ECHO_PIN 9
#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 2
#define BT_TX_PIN 10
#define BT_RX_PIN 11

// instances voor motoren en Bluetooth serial
Servo servoLeft;
Servo servoRight;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); 

// instance voor de LSM303DLHC sensor (accelerometer en kompas)
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);

void setup() {
  Serial.begin(9600);
  btSerial.begin(9600);

  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialiseer LSM303DLHC
  if (!accel.begin()) {
    Serial.println("Could not initialize the accelerometer.");
    while (1);
  }

  Serial.println("Robot is ready!");

  servoLeft.write(90);
  servoRight.write(90); 
}

void loop() {
  // Meet distance met de HC-SR04 ultrasonic sensor
  long duration, distance;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) / 29.1;  // Calculate distance in cm

  // serial print voor debugging
  Serial.print("Distance: ");
  Serial.println(distance);


  if (distance > 10) {
    servoLeft.write(90);
    servoRight.write(90);
  } else { 
    // Stop en turn 
    servoLeft.write(0);
    servoRight.write(180);
  }

  // Lees accelerometer data
  sensors_event_t event;
  accel.getEvent(&event);

  // Output accelerometer metingen voor debugging
  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(" Z: ");
  Serial.println(event.acceleration.z);

  // Check Bluetooth input 
  if (btSerial.available()) {
    char incomingByte = btSerial.read();
    Serial.print("Received Bluetooth command: ");
    Serial.println(incomingByte);

    // control using Bluetooth
    if (incomingByte == 'F') {
      servoLeft.write(90);
      servoRight.write(90);
    }
    if (incomingByte == 'B') {
      servoLeft.write(180);
      servoRight.write(0);
    }
    if (incomingByte == 'L') {
      servoLeft.write(0);
      servoRight.write(90);
    }
    if (incomingByte == 'R') {
      servoLeft.write(90);
      servoRight.write(180);
    }
  }

  delay(100);
}
