#include <Encoder.h>

// Define pins for the rotary encoder
const int clkPin = 2; // Clock pin
const int dtPin = 3;  // Data pin
const int swPin = 4;  // Switch pin

// Create an Encoder object
Encoder myEnc(clkPin, dtPin);

// Variables
int setpoint = 0; // Initial setpoint
int lastPosition = 0;

void setup() {
  Serial.begin(9600);

  // Initialize the button pin
  pinMode(swPin, INPUT_PULLUP); // Use internal pull-up resistor for the button

  Serial.println("Encoder with Library Initialized!");
}

void loop() {
  // Read the current position of the encoder
  int newPosition = myEnc.read() / 2; // Dividing by 2 to account for double increments

  // Check if the position has changed
  if (newPosition != lastPosition) {
    setpoint += (newPosition - lastPosition); // Increment setpoint by the difference
    lastPosition = newPosition;

    // Constrain the setpoint to a valid range (e.g., 0 to 150)
    setpoint = constrain(setpoint, 0, 150);

    // Print the updated setpoint
    Serial.print("Setpoint: ");
    Serial.println(setpoint);
  }

  // Check if the button is pressed
  if (digitalRead(swPin) == LOW) {
    Serial.println("Button Pressed!");
    delay(200); // Debounce delay
  }

  delay(10); // Small delay for stability
}
