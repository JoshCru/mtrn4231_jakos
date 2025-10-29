#include <PWMServo.h>

PWMServo myservo;  // create servo object to control a servo

int pos = 0;
int open = 75;
// int closed = 0; // 10g
// int closed = 1; // 20g
// int closed = 8; // 100g
// int closed = 15; // 200g
int closed = 25; // 500g
int moveRes = 1;
int currentState = 1; // 0 = closed, 1 = open (starting open)
int waitTime = 50; // ms
bool isMoving = false;


void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  pos = open;  // Assume servo is at open position
  
  // Wait for user to press Enter or Space
  Serial.println("=== GRIPPER INITIALIZATION ===");
  Serial.println("Press ENTER or SPACE to start...");
  Serial.println("==============================\n");
  
  bool started = false;
  while (!started) {
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == '\n' || input == '\r' || input == ' ') {
        started = true;
        Serial.println("Starting gripper system...");
        Serial.println("Moving to open position...\n");
      }
    }
  }
  
  // Smoothly move from closed position to open position
  while (pos < open) {
    pos += moveRes;
    if (pos > open) pos = open;
    myservo.write(pos);
    delay(waitTime);
    Serial.printf("Initializing: %d\n", pos);
  }
  
  Serial.println("Initialization complete!\n");
  delay(500);
  
  // Print controls
  Serial.println("=== GRIPPER CONTROLS ===");
  Serial.println("Press 'w' to OPEN gripper");
  Serial.println("Press 's' to CLOSE gripper");
  Serial.println("========================\n");
}

void loop() {
  // Check for user input
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    if (input == 'w' && currentState != 1) {
      openGripper();
      currentState = 1;
    } 
    else if (input == 's' && currentState != 0) {
      closeGripper();
      currentState = 0;
    }
  }
}

void openGripper() {
  isMoving = true;
  Serial.println("Opening gripper...");
  
  // Move from CURRENT position toward open
  while (pos < open) {
    // Check for state change command during movement
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 's') {
        Serial.println("Interrupted! Reversing to close...");
        closeGripper();
        currentState = 0;
        isMoving = false;
        return;
      }
    }

    pos += moveRes;
    if (pos > open) pos = open;  // Don't overshoot
    myservo.write(pos);
    delay(waitTime);
    Serial.printf("Opening: %d\n", pos);
  }
  
  Serial.println("Gripper fully opened!\n");
  isMoving = false;
}

void closeGripper() {
  isMoving = true;
  Serial.println("Closing gripper...");
  
  // Move from CURRENT position toward closed
  while (pos > closed) {
    // Check for state change command during movement
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 'w') {
        Serial.println("Interrupted! Reversing to open...");
        openGripper();
        currentState = 1;
        isMoving = false;
        return;
      }
    }
    
    pos -= moveRes;
    if (pos < closed) pos = closed;  // Don't overshoot
    myservo.write(pos);
    delay(waitTime);
    Serial.printf("Closing: %d\n", pos);
  }
  
  Serial.println("Gripper fully closed!\n");
  isMoving = false;
}