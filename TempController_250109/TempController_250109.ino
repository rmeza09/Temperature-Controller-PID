// Display Potentiometer Voltage on OLED Screen

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Define OLED display size
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SCREEN_ADDRESS 0x3C
// Initialize OLED display on I2C (A4 = SDA, A5 = SCL for Arduino Uno)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Potentiometer pin
const int potentiometerPin = A2;

// Max voltage to display
const float maxVoltage = 30.0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Potentiometer Voltage"));
  display.display();
}

void loop() {
  // Read potentiometer value
  int sensorValue = analogRead(potentiometerPin);

  // Map the analog value (0-1023) to the voltage range (0-30V)
  float voltage = map(sensorValue, 0, 1023, 0, maxVoltage * 100) / 100.0;

  // Print voltage to Serial Monitor
  Serial.print("Voltage: ");
  Serial.println(voltage, 2);

  // Display voltage on OLED screen
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Voltage"));

  // Enlarge the voltage value on the screen
  display.setTextSize(2);
  display.setCursor(0, 16);
  display.print(voltage, 2); // Display voltage with 2 decimal places
  display.println(F(" V"));
  display.display();

  delay(200); // Small delay for readability

}

