#include <Servo.h>

Servo leftServo;  // servobject links
Servo rightServo; // servobject rechts

void setup() {
  // Attach the servos to their respective pins
  leftServo.attach(9);  // Left servo op pin 9
  rightServo.attach(10); // Right servo op pin 10
}

void loop() {
  // Move both servos forward
  leftServo.write(180);  // full forw
  rightServo.write(0);   // full forw

  delay(2000); // Move forward for 2 seconds

  // Stop both servos
  leftServo.write(88);  // Neutraal
  rightServo.write(92); // Neutraal

  delay(1000); // Stop for 1 seconde

  // Move both servos backward
  leftServo.write(0);   // Full reverse
  rightServo.write(180); // Full reverse

  delay(2000); // Move backward for 2 seconds

  // Stop both servos
  leftServo.write(88);  // Neutraal
  rightServo.write(92); // Neutraal

  delay(1000); // Stop for 1 seconde
}
