#include <Servo.h>

Servo leftServo;  // Create a Servo object for the left wheel
Servo rightServo; // Create a Servo object for the right wheel

void setup() {
  // Attach the servos to their respective pins
  leftServo.attach(9);  // Left servo on pin 9
  rightServo.attach(10); // Right servo on pin 10
}

void loop() {
  // Move both servos forward
  leftServo.write(180);  // Full forward (depending on servo calibration)
  rightServo.write(0);   // Full forward (opposite direction)

  delay(2000); // Move forward for 2 seconds

  // Stop both servos
  leftServo.write(88);  // Neutral position
  rightServo.write(92); // Neutral position

  delay(1000); // Stop for 1 second

  // Move both servos backward
  leftServo.write(0);   // Full reverse
  rightServo.write(180); // Full reverse

  delay(2000); // Move backward for 2 seconds

  // Stop both servos
  leftServo.write(88);  // Neutral position
  rightServo.write(92); // Neutral position

  delay(1000); // Stop for 1 second
}
