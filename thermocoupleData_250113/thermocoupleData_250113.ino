#include <Wire.h>
#include <SparkFun_MCP9600.h>

// Create an MCP9600 object
MCP9600 thermocouple;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize I2C communication
  Wire.begin();

  // Begin communication with the MCP9600
  if (!thermocouple.begin()) {
    Serial.println("MCP9600 not detected. Check connections.");
    while (1); // Halt execution if the sensor isn't detected
  }

  // Configure the MCP9600
  thermocouple.setThermocoupleType(TYPE_K); // Use the type of thermocouple connected (e.g., TYPE_K for Type K)
  thermocouple.setFilterCoefficient(3);    // Set the filter coefficient (0-7)
  Serial.println("MCP9600 initialized!");
}

void loop() {
  // Read thermocouple temperature
  float temperature = thermocouple.getThermocoupleTemp();

  // Check if the reading is valid
  if (isnan(temperature)) {
    Serial.println("Failed to read temperature!");
  } else {
    // Print temperature to Serial Monitor
    Serial.print("Temperature: ");
    Serial.print(temperature, 2); // Display with 2 decimal places
    Serial.println(" °C");
  }

  delay(100); // Wait 1 second between readings
}
