# Temperature Control - PID Controller

This is the code for a compact temperature controlling system with a flexible heater and a J type thermocouple.
I wrote and tunned this PID controller and developed the mechatronics hardware as part of this project.
With a portable housing that can be connected to the heater, thermocouple and power source to control the system.
The final circuitry is soldered onto a modular PCB board as a cheap compact solution.

This is part of an effort to have controlled heating on a surface for benchtop testing.

Incuded is the arduino code to control I/O and the PID controller.

Completed 01/21/2025

## 🎯 Features
- Reads real-time temperature using a J-Type thermocouple and SparkFun MCP9600 Thermocouple Amplifier card.
- Uses PWM output to control a heating or cooling element with N-Type MOSFET gate modulation.
- Adjustable target temperature in Celcius.
- Two OLED displays for temperature and controller status read out.
- Manual adjustment of controller parameters (Kp, Ki, Pd) while in use.
- Live PWM value reported from feedback control response.
- I2C serial communication protocol.

## 🛠️ Hardware
- Arduino Nano Every
- SparkFun MCP9600 Thermocouple Amplifier
- J-type thermocouple
- N-Channel MOSFET 
- OMEGA 10 Watt Polyimide Flexible Heater
- 128x64 OLED display, address 0x3C
- 128x32 OLED display, address 0x3D
- 360 rotary encoder with a button for control input.
- 25 Volt Power supply.

## 🧰 Software & Libraries
- Arduino IDE
- Libraries:
  - <Adafruit_GFX.h>
  - <Adafruit_SSD1306.h>
  - <MAX6675.h> or <MAX31855.h>

