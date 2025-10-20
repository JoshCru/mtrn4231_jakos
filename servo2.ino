#include <Servo.h>

Servo myservo;

// Servo position and limits
int pos = 90;
int minPos = 180;     // Fully closed position (calibrated)
int maxPos = 0;      // Fully open position (calibrated)

int stepSize = 5;    // Degrees to move per key press
bool holdingTorque = false;  // Whether we're maintaining torque
int holdPosition = 90;       // Position to hold torque at

void setup() {
  Serial.begin(9600);
  myservo.attach(9);
  
  myservo.write(minPos);
  pos = minPos;
  
  printMenu();
}

void loop() {
  // Continuously apply torque if holding
  if (holdingTorque) {
    myservo.write(holdPosition);
  }
  
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case 'w':  // Open gripper
      case 'W':
        pos = max(pos - stepSize, maxPos);
        myservo.write(pos);
        Serial.print("Open -> ");
        Serial.println(pos);
        
        // If we reached fully open, engage constant torque
        if (pos <= maxPos) {
          holdingTorque = true;
          holdPosition = maxPos;
          Serial.println("*** HOLDING OPEN ***");
        }
        break;
        
      case 's':  // Close gripper
      case 'S':
        pos = min(pos + stepSize, minPos);
        myservo.write(pos);
        Serial.print("Close -> ");
        Serial.println(pos);
        
        // If we reached fully closed, engage constant torque
        if (pos >= minPos) {
          holdingTorque = true;
          holdPosition = minPos;
          Serial.println("*** HOLDING CLOSED ***");
        }
        break;
        
      case 'd':  // Fully open WITH HOLD
      case 'D':
        pos = maxPos;
        myservo.write(pos);
        holdingTorque = true;
        holdPosition = maxPos;
        Serial.println("FULLY OPEN - HOLDING");
        break;
        
      case 'a':  // Fully closed WITH HOLD
      case 'A':
        pos = minPos;
        myservo.write(pos);
        holdingTorque = true;
        holdPosition = minPos;
        Serial.println("FULLY CLOSED - HOLDING");
        break;
        
      case 'r':  // Release torque
      case 'R':
        holdingTorque = false;
        Serial.println("Torque released");
        break;
        
      case 'c':  // Calibrate
      case 'C':
        holdingTorque = false;
        calibrate();
        break;
        
      case 'g':  // Go to specific position
      case 'G':
        holdingTorque = false;
        goToPosition();
        break;
        
      case 'h':  // Help
      case 'H':
        printMenu();
        break;
    }
  }
}

void calibrate() {
  Serial.println("\n=== CALIBRATION ===");
  
  // Set closed position
  Serial.println("Enter CLOSED position angle (0-180):");
  while(Serial.available() == 0) {}  // Wait for input
  minPos = Serial.parseInt();
  minPos = constrain(minPos, 0, 180);
  
  myservo.write(minPos);
  pos = minPos;
  Serial.print("✓ Closed position set to: ");
  Serial.println(minPos);
  delay(1000);
  
  // Set open position
  Serial.println("\nEnter OPEN position angle (0-180):");
  while(Serial.available() == 0) {}  // Wait for input
  maxPos = Serial.parseInt();
  maxPos = constrain(maxPos, 0, 180);
  
  myservo.write(maxPos);
  pos = maxPos;
  Serial.print("✓ Open position set to: ");
  Serial.println(maxPos);
  
  Serial.println("\n=== CALIBRATION COMPLETE ===");
  Serial.print("Closed at ");
  Serial.print(minPos);
  Serial.print("°, Open at ");
  Serial.print(maxPos);
  Serial.println("°");
  
  // Return to closed position
  myservo.write(minPos);
  pos = minPos;
  
  printMenu();
}

void goToPosition() {
  Serial.println("\nEnter position (0-180):");
  while(Serial.available() == 0) {}  // Wait for input
  int targetPos = Serial.parseInt();
  targetPos = constrain(targetPos, 0, 180);
  
  myservo.write(targetPos);
  pos = targetPos;
  Serial.print("Moved to: ");
  Serial.println(targetPos);
}

void printMenu() {
  Serial.println("\n=== GRIPPER CONTROL ===");
  Serial.println("W - Open gripper (holds when fully open)");
  Serial.println("S - Close gripper (holds when fully closed)");
  Serial.println("D - Fully open + HOLD");
  Serial.println("A - Fully closed + HOLD");
  Serial.println("R - Release torque");
  Serial.println("G - Go to specific position");
  Serial.println("C - Calibrate");
  Serial.println("H - Help");
  Serial.println("=======================\n");
}