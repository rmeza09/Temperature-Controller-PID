#include <Wire.h>
#include <SparkFun_MCP9600.h>
#include <Encoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

  MCP9600 thermocouple;

  const int heaterPin = 9; // PWM pin for the heater
  // Define pins for the rotary encoder
  const int clkPin = 2; // Clock pin
  const int dtPin = 3;  // Data pin
  const int swPin = 4;  // Switch pin

  Encoder myEnc(clkPin, dtPin);
  int lastPosition = 0;

  // Define OLED display size
  #define SCREEN_WIDTH 128
  #define SCREEN_HEIGHT 32
  #define SCREEN_ADDRESS 0x3C
  // Initialize OLED display on I2C (A4 = SDA, A5 = SCL for Arduino Uno)
  Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
  unsigned long lastDisplayUpdate = 0; // Timestamp of the last display update
  const unsigned long displayInterval = 100; // 10 Hz = 100 ms interval

  float targetTemp = 0.0;
  float integralSum = 0.0;
  float prevError = 0.0;
  unsigned long prevTime = 0;

  float Kp = 38.0;
  float Ki = 0.38;
  float Kd = 2.8;

  const float maxVoltage = 28.0; // Maximum operating voltage
  const float maxTemp = 150.0;   // Maximum temperature at maxVoltage
  // User-defined power supply voltage
  float userVoltage = 25.0; // Example: 15V power source (update this as needed)

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

  // Initialize the button pin
  pinMode(swPin, INPUT_PULLUP); // Use internal pull-up resistor for the button

  Serial.println("Encoder with Library Initialized!");

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
 
}

void loop() {
  // ENCODER USER INPUT
  float Tmax = (userVoltage / maxVoltage) * maxTemp;
  // Clamp the target temperature to the calculated maximum achievable value
 
  int newPosition = myEnc.read() / 2; // Dividing by 2 to account for double increments

  // Check if the position has changed
  if (newPosition != lastPosition) {
    targetTemp += (newPosition - lastPosition); // Increment setpoint by the difference
    lastPosition = newPosition;

    // Constrain the setpoint to a valid range (e.g., 0 to 150)
    targetTemp = constrain(targetTemp, 0, Tmax);
  }

  // Check if the button is pressed
  if (digitalRead(swPin) == LOW) {
    Serial.println("Button Pressed!");
    delay(200); // Debounce delay
  }

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
  //Serial.print("prop: ");
  //Serial.print(propTerm);

  //Calculate the integral error
  integralSum += deltaTime * (error);
  //Serial.print("sum: ");
  //Serial.println(integralSum);
  integralSum = constrain(integralSum, -50, 50);
  float integTerm = Ki*(integralSum);
  //Serial.print("   integral: ");
  //Serial.print(integTerm);

  //Calculate the derivative error
  float derivTerm = Kd*(error - prevError)/deltaTime;
  //Serial.print("   deriv: ");
  //Serial.print(derivTerm);

  float output = propTerm + integTerm + derivTerm;
  
  output = constrain(output, 0, 255);
  analogWrite(heaterPin, (int)output);


 // Step 9: Log data for debugging
  Serial.print("   Setpoint: ");
  Serial.print(targetTemp);
  Serial.print(" °C, Current Temp: ");
  Serial.print(currentTemp);
  Serial.print(" °C, Output: ");
  Serial.println(output);

  prevError = error;
  prevTime = currentTime;

  // OLED DISPLAY 
  if (millis() - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = millis(); // Update the timestamp

    // Update the display
    display.clearDisplay();

    // Line 1
    display.setCursor(0, 0);
    display.println("Target Temperature");

    // Line 2
    display.setCursor(0, 8);
    display.println(targetTemp, 2);

    // Line 3
    display.setCursor(0, 16);
    display.println("Current Temperature");

    // Line 4
    display.setCursor(0, 24);
    display.println(currentTemp, 2);

    display.display(); // Render the text
  }

  delay(50);
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

















