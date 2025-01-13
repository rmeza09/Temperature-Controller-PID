const int heaterPin = 9;  // PWM pin for controlling the heater

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  pinMode(heaterPin, OUTPUT);

  Serial.println("Enter a setpoint (0 to 255) to control the heater voltage:");
}

void loop() {
  // Check if data is available in the serial input
  if (Serial.available() > 0) {
    // Read user input
    int setpoint = Serial.parseInt();

    // Ensure the setpoint is within valid range (0 to 255)
    if (setpoint < 0) setpoint = 0;
    if (setpoint > 255) setpoint = 255;

    // Output the setpoint to the heater
    analogWrite(heaterPin, setpoint);

    // Print the setpoint for confirmation
    Serial.print("Setpoint updated: ");
    Serial.println(setpoint);

    // Clear any leftover characters in the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}

