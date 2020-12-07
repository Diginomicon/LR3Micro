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

# Fuse Taps
A fuse tap of the following fuses are used to provide power and signals to the Arduino.
53P = Battery Power
58P = CJB Entertainment Relay (Retained Accessory Power)
5P  = Only powered when reverse gear selected

# Delayed Relay
In order to prevent unecessary battery drain when everything should be off, the Arduino controls a relay
which provides power to everything.  This is implemented with a single 12V power relay, a diode and a 5V
relay controlled by the Arduino.  There is a configurable delay before this relay is powered off which
keeps the Raspberry Pi running for a short while after the radio and navigation have powered off - this is
to allow Open Auto Pro to continue running for a short while for short stops without having to wait for the
Raspberry Pi to power back up.

It works by receiving power initially from the entertainment relay of the CJB - which is activated by the IHU
when the doors are unlocked or opened and before the vehicle is started.  This energizes a 12V regulated power
supply which is used to keep power variations of the vehicle from getting to any of the OAP system components.  
The power supply in turn powers on the Arduino, which sees the RAP power and in turn enables it's own RAP relay
that acts as a latched relay as log as the Arduino keeps the 5V relay energizes.  A diode 1NF822 Diode is installed
between the IHU RAP power and the Arduino's latched relay power to prevent power going back into the IHU/CJB.

When IHU/CJB RAP goes away, the Arduino starts a timer which triggers a shutdown signal to the Raspberry Pi 60
seconds before it disables it's 5V RAP relay which instantly de-energizes the entire OAP system until the next
time the IHU triggers the CJB RAP.

# Components
* Arduino Micro
* Arduino Micro expansion board (https://smile.amazon.com/gp/product/B07VQRCC8F)
* Opto-Isolated 5V Relays (https://smile.amazon.com/gp/product/B07M88JRFY)
* 5V DC-DC Buck Step Down Modules (https://smile.amazon.com/gp/product/B01HXU1C6U)
* 12V Automotive Relays (https://smile.amazon.com/gp/product/B01KVZ2MU4)
* 12V DC-DC Power Supply (https://smile.amazon.com/gp/product/B01DXEU4KA)
* Vehicle wiring harnesses so I didn't have to cut up any factory wiring