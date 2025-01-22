Hello everyone,

This is the code for a compact temperature controlling system with a flexible heater and a J type thermocouple.
I wrote and tunned this PID controller and developed the mechatronics hardware as part of this project.
I will also finally design a portable housing that can be connected to the heater, thermocouple and power source to control the system.
I'll solder the final circuitry onto a modular PCB board as a cheap compact solution.

This is part of an effort to have controlled heating on a surface for benchtop testing.

Incuded is the arduino code to control I/O and the PID controller.

This project requires a SparkFun MCP9600 Thermocouple Amplifier card and the companion SparkFun_MCP9600.h library. Two displays were used using Adafruit_GFX.h and Adafruit_SSD1306.h at addresses 0x3C and 0x3D for a 128x32 and 128x64 display respectively. The larger display has display mode capability allowing the user to provide adjustments to the controller parameters (Kp, Ki, Pd) while in use. The small display shows the current PID values when they are set.

The heater used is an OMEGA 28v flexible heater with a 25v 2A power supply connected to it. I used a mosfet gate circuit to modulate the voltage to the heater using pulse width modulation.

I used J type thermocouple wire to create a flexible sensor head.

A 360 rotary encoder with a button is used as the main input hardware to the user. The encoder allows for shifting controller parameters in discrete intervals and with good precision. Debouncing helps to intake button input to cycle between the 4 display/settings modes which are included.

Completed 01/21/2025
