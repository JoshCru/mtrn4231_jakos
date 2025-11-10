/**
 * @file gripper_controller_ros2.ino
 * @brief Teensy/Arduino gripper controller compatible with ROS2 gripper_controller_node
 *
 * Serial Protocol:
 * - Receive 'w' to open gripper
 * - Receive 's' to close gripper
 * - Send "W<weight>\n" periodically with weight measurements
 *
 * Hardware:
 * - Servo on pin 9
 * - Force sensor on pin A0 (optional)
 */

#include <PWMServo.h>

PWMServo myservo;  // create servo object to control a servo

// Gripper positions
int pos = 0;
int open_pos = 90;      // Fully open position
int closed_pos = 15;    // Closed position for 200g weight
int moveRes = 1;        // Movement resolution (degrees per step)
int waitTime = 125;     // Time between steps (ms) - controls speed
int currentState = 0;   // 0 = closed, 1 = open
bool isMoving = false;

// Force sensor (optional)
const int FORCE_SENSOR_PIN = A0;
bool has_force_sensor = false;
unsigned long last_weight_send = 0;
const unsigned long WEIGHT_SEND_INTERVAL = 50;  // Send weight every 50ms (20Hz)

void setup() {
  Serial.begin(115200);
  myservo.attach(9);

  // Start in closed position
  pos = closed_pos;
  myservo.write(pos);
  currentState = 0;

  // Check if force sensor is connected
  pinMode(FORCE_SENSOR_PIN, INPUT);
  int sensor_reading = analogRead(FORCE_SENSOR_PIN);
  has_force_sensor = (sensor_reading > 0);  // Simple check

  // Send startup message
  Serial.println("Gripper controller ready");
  if (has_force_sensor) {
    Serial.println("Force sensor detected");
  } else {
    Serial.println("No force sensor (will send simulated values)");
  }
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == 'w' && currentState != 1 && !isMoving) {
      // Open gripper
      openGripper();
      currentState = 1;
    }
    else if (input == 's' && currentState != 0 && !isMoving) {
      // Close gripper
      closeGripper();
      currentState = 0;
    }
    // Ignore invalid commands or commands during movement
  }

  // Periodically send weight data
  unsigned long current_time = millis();
  if (current_time - last_weight_send >= WEIGHT_SEND_INTERVAL) {
    sendWeightData();
    last_weight_send = current_time;
  }
}

void openGripper() {
  isMoving = true;

  while (pos < open_pos) {
    // Check for interruption
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 's') {
        // Interrupted - reverse to close
        closeGripper();
        currentState = 0;
        isMoving = false;
        return;
      }
    }

    pos += moveRes;
    if (pos > open_pos) pos = open_pos;
    myservo.write(pos);
    delay(waitTime);
  }

  isMoving = false;
}

void closeGripper() {
  isMoving = true;

  while (pos > closed_pos) {
    // Check for interruption
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (input == 'w') {
        // Interrupted - reverse to open
        openGripper();
        currentState = 1;
        isMoving = false;
        return;
      }
    }

    pos -= moveRes;
    if (pos < closed_pos) pos = closed_pos;
    myservo.write(pos);
    delay(waitTime);
  }

  isMoving = false;
}

void sendWeightData() {
  float weight = 0.0;

  if (has_force_sensor) {
    // Read actual force sensor
    int sensor_value = analogRead(FORCE_SENSOR_PIN);
    // Convert to weight (you'll need to calibrate this)
    // This is a placeholder conversion
    weight = sensor_value * 0.1;  // Adjust calibration factor
  } else {
    // Simulated weight based on gripper position
    if (currentState == 0) {  // Closed
      weight = 150.0 + random(-10, 10);  // Simulate 140-160g object
    } else {  // Open
      weight = 0.0 + random(0, 5);  // Small noise when open
    }
  }

  // Send weight in format: W<value>\n
  Serial.print("W");
  Serial.println(weight, 1);  // 1 decimal place
}
