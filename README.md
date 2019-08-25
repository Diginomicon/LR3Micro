# LR3Micro
Manange the vehicle signals from a Land Rover LR3 and convert them to appropriate triggers or keyboard presses for OpenAuto Pro.  Implemented on an Arduino Micro.

Part of an OpenAuto implemtation to inject bluetooth and Android Auto into the OEM navigation screen, while retaining all OEM functionality.

# Features
Provides power management by acting a delayed relay.

Handles steering wheel resistive ladder input via built-in ADC and emulates it back to the integrated head unit using Adafruit MCP4725 DAC. This enables stealing some of the buttons based on the active video input to the navigation screen to route them to the arduino. Also used to manually toggle rear-view camera on/off and to mute the stereo regardless of video mode. 

Works with a 4 input optocoupler to safely isolate 12V signals from the car to 3V3 signals for the arduino.
* Retained Accessory Power sensing
* Reverse Signal sensing
* GVIF RGB Input sensing
* Headlights on/off sensing
