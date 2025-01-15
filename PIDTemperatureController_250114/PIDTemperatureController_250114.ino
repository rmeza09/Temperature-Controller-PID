#include <Wire.h>
#include <SparkFun_MCP9600.h>

MCP9600 thermocouple;

  const int heaterPin = 9; // PWM pin for the heater
  const int potentiometerPin = A2;

  float targetTemp = 0.0;
  float integralSum = 0.0;
  float prevError = 0.0;
  unsigned long prevTime = 0;

  float Kp = 2.0;
  float Ki = 0;
  float Kd = 0.1;

  const float maxVoltage = 28.0; // Maximum operating voltage
  const float maxTemp = 149.0;   // Maximum temperature at maxVoltage
  // User-defined power supply voltage
  float userVoltage = 15.0; // Example: 15V power source (update this as needed)

  float prevSetpoint = targetTemp;
  
void setup() {
  Serial.begin(9600);
  Wire.begin();
  // Begin communication with the MCP9600
  if (!thermocouple.begin()) {
    Serial.println("MCP9600 not detected. Check connections.");
    while (1); // Stop execution if the sensor isn't detected
  }
  thermocouple.setThermocoupleType(TYPE_K); // Use the type of thermocouple connected (e.g., TYPE_K for Type K)
  thermocouple.setFilterCoefficient(3); 

  pinMode(heaterPin, OUTPUT);

  Serial.println("PID Control with MCP9600 Initialized");

 
}

void loop() {
  // POTENTIOMETER INPUT
  int userSetTemp = analogRead(potentiometerPin);
  // Map the analog value (0-1023) to the temperature range (0-maxTemp)
  float targetTemp = map(userSetTemp, 0, 1023, 25, maxTemp);
  // Calculate the maximum achievable temperature based on userVoltage
  float Tmax = (userVoltage / maxVoltage) * maxTemp;
  // Clamp the target temperature to the calculated maximum achievable value
  if (targetTemp > Tmax) {
      targetTemp = Tmax;
  }
  //Serial.print("target: ");
  //Serial.println(targetTemp);

  if (targetTemp != prevSetpoint) {
    integralSum = 0.0; // Reset the accumulated error
    prevSetpoint = targetTemp; // Update the previous setpoint
    Serial.println("Setpoint changed. Resetting integral term.");
}

  // PID CONTROLLER
  float currentTemp = readSensorTemp();
  //Serial.print("temp: "); 
  //Serial.println(currentTemp);
  unsigned long currentTime = millis();
  float error = targetTemp - currentTemp;
  //Serial.print("Error: ");
  //Serial.println(error);
  float deltaTime = (currentTime - prevTime)/1000.0;
  //Serial.print("time delta: ");
  //Serial.println(deltaTime);
  // Calculate proportional error
  float propTerm = Kp*error;
  Serial.print("prop: ");
  Serial.print(propTerm);

  //Calculate the integral error
  float midpointInterp = prevError + ((deltaTime/2.0 - prevTime)/deltaTime)*(error-prevError); //midpoint interpolation
  //Serial.print("interpMid: ");
  //Serial.println(midpointInterp);
  integralSum += deltaTime * (midpointInterp); //midpoint rule integration
  //Serial.print("sum: ");
  //Serial.println(integralSum);
  float integTerm = Ki*(integralSum);
  Serial.print("   integral: ");
  Serial.print(integTerm);

  //Calculate the derivative error
  float derivTerm = Kd*(error - prevError)/deltaTime;
  Serial.print("   deriv: ");
  Serial.println(derivTerm);

  float output = propTerm + integTerm + derivTerm;
  
  output = constrain(output, 0, 255);
  analogWrite(heaterPin, (int)output);


 // Step 9: Log data for debugging
  Serial.print("Setpoint: ");
  Serial.print(targetTemp);
  Serial.print(" °C, Current Temp: ");
  Serial.print(currentTemp);
  Serial.print(" °C, Output: ");
  Serial.println(output);


  prevError = error;
  prevTime = currentTime;

  delay(1000);
}

float readSensorTemp(){
  // THERMOCOUPLE READING
  float currentTemp = thermocouple.getThermocoupleTemp();
  if (isnan(currentTemp)) {
    Serial.println("Failed to read temperature!");
  } else {
   //Serial.print("Temperature: ");
    //Serial.print(currentTemp, 2); // Display with 2 decimal places
    //Serial.println(" °C");
  }
  return currentTemp;
}

















