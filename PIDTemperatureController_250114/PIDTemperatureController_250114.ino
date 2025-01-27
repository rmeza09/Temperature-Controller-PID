#include <Wire.h>
#include <SparkFun_MCP9600.h>
#include <Encoder.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

MCP9600 thermocouple;

const int heaterPin = 6; // PWM pin for the heater

// Rotary Encoder Pins
const int clkPin = 2; // Clock pin
const int dtPin = 3;  // Data pin
const int swPin = 4;  // Button pin

Encoder myEnc(clkPin, dtPin);
int lastPosition = 0;

// Define OLED display sizes and addresses
#define SCREEN_WIDTH_1 128
#define SCREEN_HEIGHT_1 32
#define SCREEN_WIDTH_2 128
#define SCREEN_HEIGHT_2 64

#define SCREEN_ADDRESS_1 0x3C // Small OLED (PID values)
#define SCREEN_ADDRESS_2 0x3D // Large OLED (Temperature info)

// Initialize OLED displays
Adafruit_SSD1306 display1(SCREEN_WIDTH_1, SCREEN_HEIGHT_1, &Wire, -1); // 128x32 (Kp, Ki, Kd)
Adafruit_SSD1306 display2(SCREEN_WIDTH_2, SCREEN_HEIGHT_2, &Wire, -1); // 128x64 (Temperature info)
int displayMode = 0;

unsigned long lastDisplayUpdate = 0; // Timestamp for OLED updates
const unsigned long displayInterval = 100; // 10 Hz = 100 ms interval

float targetTemp = 0.0;
float integralSum = 0.0;
float prevError = 0.0;
unsigned long prevTime = 0;

// PID Constants
float Kp = 8.0;
float Ki = 0.38;
float Kd = 2.8;

const float maxVoltage = 28.0; // Maximum operating voltage
const float maxTemp = 150.0;   // Maximum temperature at maxVoltage

// User-defined power supply voltage
float userVoltage = 25.0;

float prevSetpoint = targetTemp;

// Button debouncing
unsigned long lastButtonPress = 0;
const unsigned long debounceDelay = 200; // 200 ms debounce

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize MCP9600 Thermocouple
  if (!thermocouple.begin()) {
    Serial.println("MCP9600 not detected. Check connections.");
    while (1);
  }
  thermocouple.setThermocoupleType(TYPE_J);
  thermocouple.setFilterCoefficient(3);

  pinMode(heaterPin, OUTPUT);

  Serial.println("PID Control with MCP9600 Initialized");

  // Initialize Rotary Encoder Button
  pinMode(swPin, INPUT_PULLUP); // Enable internal pull-up resistor
  Serial.println("Encoder with Library Initialized!");

  // Initialize OLED 1 (128x32) for PID values
  if (!display1.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS_1)) {
    Serial.println(F("OLED 1 allocation failed"));
    for (;;);
  }

  // Initialize OLED 2 (128x64) for temperature display
  if (!display2.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS_2)) {
    Serial.println(F("OLED 2 allocation failed"));
    for (;;);
  }

  display1.clearDisplay();
  display2.clearDisplay();
}

void loop() {
  // ENCODER USER INPUT
  float Tmax = (userVoltage / maxVoltage) * maxTemp;

  int newPosition = myEnc.read() / 2; // Adjust for encoder double step
  if (newPosition != lastPosition) {
        if (displayMode == 0) {
            targetTemp += (newPosition - lastPosition);
            targetTemp = constrain(targetTemp, 0, 150);
        } else if (displayMode == 1) {
            Kp += (newPosition - lastPosition) * 0.01;
        } else if (displayMode == 2) {
            Ki += (newPosition - lastPosition) * 0.01;
        } else if (displayMode == 3) {
            Kd += (newPosition - lastPosition) * 0.01;
        }
        lastPosition = newPosition;
    }

  // **Check Button Press with Debouncing**
  if (digitalRead(swPin) == LOW) {
    if (millis() - lastButtonPress > debounceDelay) { // Debounce check
      displayMode = (displayMode + 1) % 4;
            Serial.print("Switched to mode: ");
            Serial.println(displayMode);
      lastButtonPress = millis(); // Update last button press time
    }
  }

  if (targetTemp != prevSetpoint) {
    integralSum = 0.0;
    prevSetpoint = targetTemp;
    Serial.println("Setpoint changed. Resetting integral term.");
  }

  // PID CONTROLLER
  float currentTemp = readSensorTemp();
  unsigned long currentTime = millis();
  float error = targetTemp - currentTemp;
  float deltaTime = (currentTime - prevTime) / 1000.0;

  // PID Calculations
  float propTerm = Kp * error;
  integralSum += deltaTime * error;
  integralSum = constrain(integralSum, -50, 50);
  float integTerm = Ki * integralSum;
  float derivTerm = Kd * (error - prevError) / deltaTime;

  float output = propTerm + integTerm + derivTerm;

  // if in setttings close the output gate
  output = constrain(output, 0, 255);
  if (displayMode!=0){
    output = 0;
  }
  // apply the pwm signal to the heater
  analogWrite(heaterPin, (int)output);

  Serial.print("Setpoint: ");
  Serial.print(targetTemp);
  Serial.print(" °C, Current Temp: ");
  Serial.print(currentTemp);
  Serial.print(" °C, Output: ");
  Serial.println(output);

  prevError = error;
  prevTime = currentTime;

  // OLED DISPLAY UPDATE at 10 Hz with four different display modes
  if (millis() - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = millis();

    // clear displays routinely 
    display2.clearDisplay();
    display2.setTextSize(2); // Enlarged text (16 pixels per line)
    display2.setTextColor(SSD1306_WHITE);
    display1.clearDisplay();
    
    display2.setCursor(0, 0);

    switch(displayMode){
      case 0:
        display2.println("Target:");
        display2.setCursor(0, 16);
        display2.print(targetTemp, 2);
        display2.println("  C");

        display2.setCursor(0, 32);
        display2.println("Current:");
        display2.setCursor(0, 48);
        display2.print(currentTemp, 2);
        display2.println("  C");

        display2.display();

        // Update 128x32 OLED (PID Values Display)
        display1.clearDisplay();
        display1.setTextSize(1); // text size (8 pixels per line)
        display1.setTextColor(SSD1306_WHITE);

        display1.setCursor(0, 0);
        display1.print("Output PWM: ");
        display1.print(output, 2);
        display1.setCursor(0, 8);
        display1.print("Kp: ");
        display1.println(Kp, 2);
        display1.setCursor(0, 16);
        display1.print("Ki: ");
        display1.println(Ki, 2);
        display1.setCursor(0, 24);
        display1.print("Kd: ");
        display1.println(Kd, 2);
        display1.display();
        break;

        case 1:
            display2.setCursor(0, 16);
            display2.println("Edit Kp:");
            display2.setCursor(0, 32);
            display2.println(Kp, 2);
            display2.display();

            display1.setTextSize(2);
            display1.setCursor(10, 10);
            display1.println("SETTINGS");
            display1.display();

        break;

        case 2:
            display2.setCursor(0, 16);
            display2.println("Edit Ki:");
            display2.setCursor(0, 32);
            display2.println(Ki, 2);
            display2.display();

            display1.setTextSize(2);
            display1.setCursor(10, 10);
            display1.println("SETTINGS");
            display1.display();
            
        break;

        case 3:
            display2.setCursor(0, 16);
            display2.println("Edit Kd:");
            display2.setCursor(0, 32);
            display2.println(Kd, 2);
            display2.display();

            display1.setTextSize(2);
            display1.setCursor(10, 10);
            display1.println("SETTINGS");
            display1.display();
            
        break;
    }
  }

  delay(50);
}

float readSensorTemp() {
  float currentTemp = thermocouple.getThermocoupleTemp();
  if (isnan(currentTemp)) {
    Serial.println("Failed to read temperature!");
  }
  return currentTemp;
}


