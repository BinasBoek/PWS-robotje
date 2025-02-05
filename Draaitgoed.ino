#include <Servo.h>
#include <SoftwareSerial.h>


#define SERVO_LEFT_PIN 4
#define SERVO_RIGHT_PIN 2
#define TRIG_PIN 8
#define ECHO_PIN 9
#define BT_TX_PIN 11
#define BT_RX_PIN 10


Servo servoLeft;
Servo servoRight;


SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// HC-SR04 sensor variables
long duration;
int distance;

// Command variables
char command;

void setup() {
  // Start serial communication (optional)
  Serial.begin(9600);
  btSerial.begin(9600);

  
  servoLeft.attach(SERVO_LEFT_PIN);
  servoRight.attach(SERVO_RIGHT_PIN);

  // Initialize the trigger and echo pins for HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Set initial servo positions to forward
  servoLeft.writeMicroseconds(1511);
  servoRight.writeMicroseconds(1511);

  Serial.println("Robot is ready to move.");
}

void loop() {
  // Check if Bluetooth data is available
  if (btSerial.available()) {
    command = btSerial.read(); // Read the incoming Bluetooth command
    Serial.print("Received command: ");
    Serial.println(command);

    // Handle Bluetooth command
    if (command == 'F') {
      moveForward();
    }
    else if (command == 'S') {
      stopMovement();
    }
    else if (command == 'B') {
      moveBackward();
    }
    else if (command == 'L') {
      turnLeft();
    }
    else if (command == 'R') {
      turnRight();
    }
    else {
      Serial.println("Unknown command");
    }
  }


  distance = getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  
  if (distance < 20) {
    stopMovement();
    Serial.println("Obstacle detected! Stopping.");
  }

  delay(100);
}

// Function to get the distance from the HC-SR04 sensor
int getDistance() {
  // Send a 10ms pulse to trigger the sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance (in cm)
  distance = duration * 0.0344 / 2;
  return distance;
}

void moveForward() {
  servoLeft.write(800);
  servoRight.write(1600);
  Serial.println("Moving Forward");
}


void moveBackward() {
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(800);
  Serial.println("Moving Backward");
}


void stopMovement() {
  servoLeft.writeMicroseconds(1511);
  servoRight.writeMicroseconds(1511);
  Serial.println("Stopped");
}


void turnLeft() {
  servoLeft.writeMicroseconds(800);
  servoRight.writeMicroseconds(800);
  Serial.println("Turning Left");
}


void turnRight() {
  servoLeft.writeMicroseconds(1600);
  servoRight.writeMicroseconds(1600);
  Serial.println("Turning Right");
}

