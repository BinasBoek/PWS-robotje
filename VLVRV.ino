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

// instances servo motoren en Bluetooth serial
Servo servoLeft;
Servo servoRight;
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // Bluetooth module on RX/TX

// instance voor the LSM303DLHC sensor (accelerometer and compass)
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);

void setup() {
    Serial.begin(9600);
    btSerial.begin(9600);  // Bluetooth baud rate

    // Attach servo motors
    servoLeft.attach(SERVO_LEFT_PIN);
    servoRight.attach(SERVO_RIGHT_PIN);

    // Set pin modes ultrasonic sensor
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialiseer LSM303DLHC
    if (!accel.begin()) {
        Serial.println("Could not initialize the accelerometer.");
        while (1);
    }

    Serial.println("Robot is ready!");

    stopMotors(); // Ensure motors are stopped at start

    // Execute the movement sequence
    executeMovementSequence();
}

void loop() {
    // Loop does nothing after executing the movement sequence once
}

// === Movement Sequence Function ===
void executeMovementSequence() {
    moveForward(5000);
    turnLeft(90);
    moveForward(7000);
    turnRight(90);
    moveForward(2000);
    stopMotors();
}

// === Motion Functions ===
void moveForward(int duration) {
    Serial.println("Moving Forward");
    servoLeft.write(90);
    servoRight.write(90);
    delay(duration);
    stopMotors();
}

void turnLeft(int degrees) {
    Serial.println("Turning Left");
    int targetHeading = (getCompassHeading() - degrees + 360) % 360;
    servoLeft.write(0);
    servoRight.write(90);
    while (abs(getCompassHeading() - targetHeading) > 5) {
        delay(50);
    }
    stopMotors();
}

void turnRight(int degrees) {
    Serial.println("Turning Right");
    int targetHeading = (getCompassHeading() + degrees) % 360;
    servoLeft.write(90);
    servoRight.write(180);
    while (abs(getCompassHeading() - targetHeading) > 5) {
        delay(50);
    }
    stopMotors();
}

void stopMotors() {
    Serial.println("Stopping");
    servoLeft.writeMicroseconds(1511);
    servoRight.writeMicroseconds(1511);
}

// === Sensor Functions ===
int getCompassHeading() {
    sensors_event_t event;
    accel.getEvent(&event);
    float heading = atan2(event.acceleration.y, event.acceleration.x) * 180 / PI;
    if (heading < 0) heading += 360;
    return (int)heading;
}
