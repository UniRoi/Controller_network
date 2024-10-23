#include <Arduino.h>
#include <SoftwareSerial.h>


SoftwareSerial mySerial(2, 3);

// Define the analog pin you're using
const int analogPin = A0;  // Analog pin to read the voltage (e.g., A0)

// Variables to store the analog value and voltage
int analogValue = 0;
float voltage = 0.0;

// The reference voltage (usually 5V for Arduino Nano)
const float referenceVoltage = 5.0;

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
}

void update_sensor() {
  // Read the analog value (0 to 1023)
  analogValue = analogRead(analogPin);
  
  // Convert the analog value to voltage
  voltage = (analogValue * referenceVoltage) / 1023.0;

  // Print the voltage to the Serial Monitor
  Serial.print("Voltage: ");
  Serial.println(voltage);

  // Wait for 20 milliseconds before the next reading
  
}

void loop(){
  update_sensor();

  delay(20);

}
