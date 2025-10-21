/**
 * Arduino Gripper Controller for ROS2 Sort System
 *
 * Hardware:
 * - Servo motor on pin 9
 * - Force/weight sensor on analog pin A0
 * - Serial communication at 115200 baud
 *
 * Protocol:
 * - Receive: G<angle>\n (e.g., G90 for 90 degrees)
 * - Send: W<weight>\n (e.g., W150.5 for 150.5 grams)
 */

#include <Servo.h>

// Pin definitions
const int SERVO_PIN = 9;
const int FORCE_SENSOR_PIN = A0;

// Servo object
Servo gripperServo;

// Variables
int currentAngle = 0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 50; // ms (20 Hz)

// Calibration constants (adjust based on your sensor)
const float CALIBRATION_FACTOR = 1.0;
const float ZERO_OFFSET = 0.0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Attach servo
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(0); // Start with gripper open

  // Initialize analog pin
  pinMode(FORCE_SENSOR_PIN, INPUT);

  Serial.println("Gripper Controller Ready");
}

void loop() {
  // Check for incoming commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Periodically read and send force sensor data
  unsigned long currentTime = millis();
  if (currentTime - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = currentTime;
    sendForceSensorData();
  }
}

void processCommand(String command) {
  command.trim();

  if (command.length() < 2) {
    return;
  }

  char cmdType = command.charAt(0);
  String value = command.substring(1);

  switch (cmdType) {
    case 'G': // Gripper command
    case 'g':
      {
        int angle = value.toInt();
        angle = constrain(angle, 0, 180);
        gripperServo.write(angle);
        currentAngle = angle;
        Serial.print("OK:G");
        Serial.println(angle);
      }
      break;

    case 'C': // Calibrate
    case 'c':
      calibrateSensor();
      break;

    default:
      Serial.println("ERROR:Unknown command");
      break;
  }
}

void sendForceSensorData() {
  // Read analog sensor
  int rawValue = analogRead(FORCE_SENSOR_PIN);

  // Convert to weight (grams)
  // TODO: Adjust this formula based on your specific force sensor
  // This is a placeholder linear conversion
  float voltage = rawValue * (5.0 / 1023.0);
  float weight = (voltage - ZERO_OFFSET) * CALIBRATION_FACTOR * 100.0;

  // Clamp to reasonable range
  weight = constrain(weight, 0, 1000);

  // Send over serial
  Serial.print("W");
  Serial.println(weight, 1);
}

void calibrateSensor() {
  // Tare the sensor (set current reading as zero)
  Serial.println("Calibrating... Remove all weight from sensor.");
  delay(2000);

  // Take multiple readings and average
  long sum = 0;
  const int numReadings = 100;

  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(FORCE_SENSOR_PIN);
    delay(10);
  }

  int zeroReading = sum / numReadings;

  Serial.print("Calibration complete. Zero reading: ");
  Serial.println(zeroReading);
  Serial.println("OK:Calibrated");
}

// Optional: Advanced filtering for smoother weight readings
float exponentialFilter(float newValue, float previousValue, float alpha = 0.2) {
  return alpha * newValue + (1 - alpha) * previousValue;
}
