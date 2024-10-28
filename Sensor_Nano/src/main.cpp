#include <Arduino.h>
#include <SoftwareSerial.h>


const int analogPin = A0;           // Analog pin to read the voltage
const float referenceVoltage = 5.0; // Reference voltage
const float thresholdVoltage = 3.0; // Threshold for light/dark detection
const int sampleInterval = 20;      // Sample interval in milliseconds
const unsigned long timeout = 5000; // 5 seconds timeout in milliseconds

int analogValue = 0;
float voltage = 0.0;
bool isDark = false;                // Tracks if the sensor is in a dark state
unsigned long lastRoundTime = 0;    // Time of the last dark-to-light transition
unsigned long firstRoundTime = 0;   // Time of the first detected round after timeout
float rpm = 0.0;                    // Calculated RPM value
bool timeoutOccurred = false;       // Tracks if a timeout has reset the RPM

void setup() {
  Serial.begin(9600);  // Initialize serial communication
}

void updateSensor() {
  // Read the analog value and convert it to voltage
  analogValue = analogRead(analogPin);
  voltage = (analogValue * referenceVoltage) / 1023.0;

  // Check if the sensor detects "dark" or "light"
  if (voltage < thresholdVoltage) { // Dark state detected
    if (!isDark) {                  // If previously light, detect a round
      isDark = true;                // Set state to dark

      // Check if we are restarting after a timeout
      if (timeoutOccurred) {        
        firstRoundTime = millis();  // Start new timing interval
        timeoutOccurred = false;    // Clear the timeout flag
        lastRoundTime = firstRoundTime; // Reset last round time for next interval
      } else {
        // Calculate RPM based on time between two consecutive dark states
        unsigned long currentTime = millis();
        unsigned long timeDifference = currentTime - lastRoundTime;
        rpm = (60000.0 / timeDifference); // Calculate RPM
        lastRoundTime = currentTime;      // Update last round time
      }
    }
  } else {                     // Light state detected
    isDark = false;            // Reset state to light
  }

  // Timeout: Check if more than 5 seconds have passed since the last round
  if (millis() - lastRoundTime >= timeout) {
    rpm = 0.0;                 // Set RPM to zero if timeout occurs
    timeoutOccurred = true;    // Set timeout flag to reset calculation
  }

  // Print results
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, ");
  Serial.print("RPM: ");
  Serial.println(rpm);

  delay(sampleInterval);       // Wait for the next sample
}

void loop() {
  updateSensor();              // Update sensor reading and calculate RPM
}
