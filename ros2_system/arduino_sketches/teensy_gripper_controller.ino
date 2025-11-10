/**
 * Teensy 4.1 Gripper Controller for ROS2 Sort System
 *
 * Hardware:
 * - Teensy 4.1 board
 * - Servo motor on pin 9
 * - Force/weight sensor on analog pin A0
 * - Serial communication via USB at 115200 baud
 *
 * Protocol:
 * - Receive: G<angle>\n (e.g., G90 for 90 degrees)
 * - Send: W<weight>\n (e.g., W150.5 for 150.5 grams)
 *
 * Teensy 4.1 Features Used:
 * - Higher precision ADC (12-bit)
 * - Faster processing for better sensor filtering
 * - Native USB serial for reliable communication
 */

#include <Servo.h>

// Pin definitions
const int SERVO_PIN = 9;
const int FORCE_SENSOR_PIN = A0;  // Teensy A0 = pin 14

// Servo object
Servo gripperServo;

// Variables
int currentAngle = 0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 50; // ms (20 Hz)

// Calibration constants (adjust based on your specific sensor)
// These will be set during calibration or can be pre-configured
float calibrationFactor = 1.0;
float zeroOffset = 0.0;

// Exponential filter for smoother readings
float filteredWeight = 0.0;
const float ALPHA = 0.2; // Filter coefficient (0-1, lower = more filtering)

void setup() {
  // Initialize USB serial communication
  Serial.begin(115200);

  // Wait for serial connection (Teensy feature - can remove if not needed)
  // Uncomment the next line if you want to wait for serial monitor before starting
  // while (!Serial && millis() < 3000);  // Wait up to 3 seconds for Serial

  // Attach servo
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(0); // Start with gripper open

  // Configure ADC for better precision (Teensy 4.1 specific)
  analogReadResolution(12); // 12-bit ADC (0-4095)
  analogReadAveraging(16);  // Average 16 samples for noise reduction

  // Initialize analog pin
  pinMode(FORCE_SENSOR_PIN, INPUT);

  // Startup message
  Serial.println("Teensy 4.1 Gripper Controller Ready");
  Serial.println("Commands: G<angle> (0-180), C (calibrate)");

  // Initial calibration
  delay(500);
  calibrateSensor();
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

  if (command.length() < 1) {
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

    case 'S': // Status request
    case 's':
      printStatus();
      break;

    default:
      Serial.println("ERROR:Unknown command");
      Serial.println("Available: G<angle>, C (calibrate), S (status)");
      break;
  }
}

void sendForceSensorData() {
  // Read analog sensor with Teensy's 12-bit ADC
  int rawValue = analogRead(FORCE_SENSOR_PIN);

  // Convert to voltage (Teensy 4.1 uses 3.3V reference)
  float voltage = rawValue * (3.3 / 4095.0);

  // Convert to weight (grams)
  // TODO: Adjust this formula based on your specific force sensor
  // This is a placeholder linear conversion
  // Common sensors: FSR402, Load Cell with HX711, etc.
  float weight = (voltage - zeroOffset) * calibrationFactor;

  // Apply exponential filter for smoother readings
  filteredWeight = ALPHA * weight + (1.0 - ALPHA) * filteredWeight;

  // Clamp to reasonable range
  filteredWeight = constrain(filteredWeight, 0, 1000);

  // Send over serial
  Serial.print("W");
  Serial.println(filteredWeight, 1);  // 1 decimal place
}

void calibrateSensor() {
  Serial.println("Calibrating... Remove all weight from sensor.");
  delay(2000);

  // Take multiple readings and average (using Teensy's fast ADC)
  long sum = 0;
  const int numReadings = 200;  // More samples due to fast Teensy

  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(FORCE_SENSOR_PIN);
    delay(5);
  }

  int zeroReading = sum / numReadings;

  // Convert to voltage for zero offset
  zeroOffset = zeroReading * (3.3 / 4095.0);

  // Reset filtered weight
  filteredWeight = 0.0;

  Serial.print("Calibration complete. Zero reading: ");
  Serial.print(zeroReading);
  Serial.print(" ADC (");
  Serial.print(zeroOffset, 4);
  Serial.println("V)");
  Serial.println("OK:Calibrated");
}

void printStatus() {
  Serial.println("=== Teensy Gripper Status ===");
  Serial.print("Gripper Angle: ");
  Serial.print(currentAngle);
  Serial.println(" degrees");
  Serial.print("Current Weight: ");
  Serial.print(filteredWeight, 1);
  Serial.println(" grams");
  Serial.print("Zero Offset: ");
  Serial.print(zeroOffset, 4);
  Serial.println(" V");
  Serial.print("Calibration Factor: ");
  Serial.println(calibrationFactor, 4);
  Serial.println("==========================");
}

// Advanced: Implement two-point calibration
// Call this with known weights to set calibration factor
void calibrateWithKnownWeight(float knownWeight) {
  Serial.print("Place ");
  Serial.print(knownWeight);
  Serial.println("g weight on sensor and press any key...");

  // Wait for user input
  while (!Serial.available()) {
    delay(100);
  }
  Serial.read(); // Clear the buffer

  // Take reading
  long sum = 0;
  const int numReadings = 200;

  for (int i = 0; i < numReadings; i++) {
    sum += analogRead(FORCE_SENSOR_PIN);
    delay(5);
  }

  int avgReading = sum / numReadings;
  float voltage = avgReading * (3.3 / 4095.0);

  // Calculate calibration factor
  // weight = (voltage - zeroOffset) * calibrationFactor
  // calibrationFactor = weight / (voltage - zeroOffset)
  if (voltage - zeroOffset > 0.01) { // Avoid division by near-zero
    calibrationFactor = knownWeight / (voltage - zeroOffset);
    Serial.print("Calibration factor updated: ");
    Serial.println(calibrationFactor, 4);
  } else {
    Serial.println("ERROR: Voltage reading too close to zero offset");
  }
}
