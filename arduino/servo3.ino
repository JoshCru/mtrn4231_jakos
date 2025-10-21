#include <Servo.h>

Servo myservo;  // create servo object to control a servo

int open = 90;
int closed = 0;
unsigned long lastTorqueUpdate = 0;
const int torqueInterval = 15;  // Apply torque every 15ms
int currentPosition = open;     // Track current gripper state

void setup() {
  Serial.begin(9600);           // Initialize serial communication
  myservo.attach(9);            // Attaches the servo on pin 9 to the servo object
  myservo.write(open);
  currentPosition = open;
  delay(500);
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    if (command == 'w' || command == 'W') {
      currentPosition = closed;  // Close gripper
    }
    else if (command == 's' || command == 'S') {
      currentPosition = open;    // Open gripper
    }
  }
  
  // Apply constant torque every 15ms
  if (millis() - lastTorqueUpdate >= torqueInterval) {
    myservo.write(currentPosition);
    lastTorqueUpdate = millis();
  }
}