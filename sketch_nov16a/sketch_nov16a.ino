// PID Heater Control for Arduino

#include <PID_v1.h>

// Define the pins
const int heaterPin = 9;  // PWM pin to control the heater
const int tempSensorPin = A0;  // Analog pin for temperature sensor (not used for now)

// Define variables for PID
double setPoint = 25.0; // Target temperature in degrees Celsius (initially 25.0)
double input;           // Current temperature (simulated for now)
double output;          // PID output to control the heater

// PID Tuning Parameters
double Kp = 2.0;  // Proportional gain
double Ki = 5.0;  // Integral gain
double Kd = 1.0;  // Derivative gain

// Initialize PID
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(heaterPin, OUTPUT);

  // Initialize the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range for Arduino (0-255)

  Serial.println("PID Heater Control Started");
}

void loop() {
  // Simulate the current temperature
  input = getSimulatedTemperature();

  // Compute PID
  myPID.Compute();

  // Output to the heater (for now, we just print the value)
  analogWrite(heaterPin, output);
  
  // Debug output
  Serial.print("Set Point: ");
  Serial.print(setPoint);
  Serial.print(" | Current Temp: ");
  Serial.print(input);
  Serial.print(" | PID Output: ");
  Serial.println(output);

  // Simulate a change in temperature target for testing
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    setPoint = inputString.toFloat();
    Serial.print("Updated Set Point: ");
    Serial.println(setPoint);
  }

  delay(1000); // Update every second
}

// Function to simulate temperature changes (replace with actual sensor later)
double getSimulatedTemperature() {
  static double simulatedTemp = 20.0; // Initial simulated temperature

  // Adjust simulated temperature toward setPoint for testing
  if (simulatedTemp < setPoint) {
    simulatedTemp += 0.1; // Heating up
  } else if (simulatedTemp > setPoint) {
    simulatedTemp -= 0.1; // Cooling down
  }

  return simulatedTemp;
}
