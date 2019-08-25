/**************************************************************************/
/*
    @file     LR3Micro.ino
    @author   Sean Settle
    @license  Creative Commons Attribution ShareAlike (CC BY-SA)

    Manange the vehicle signals from a Land Rover LR3 and convert them
    to appropriate triggers or keyboard presses for OpenAuto Pro.

    Designed for Arduino Micro microcontroller.

    Limits the use of delay function to prevent blocking other signals.

    A0 = INPUT from Factory SWC+
    2/SDA & 3/SCL = Adafruit MCP4725 I2C
    +5V = POWER to SWC Voltage Divider & MCP4725 VDD
    GND = SWC- & MCP4725 GND & Vehicle Body
    4/A6 = DEBUG if low, INPUT_PULLUP
    5 = INPUT from Optocoupler Entertainment Relay (RAP)
    6/A7 = INPUT from Optocoupler Reverse Signal
    7 = INPUT from Optocoupler RGB Signal
    8 = OUTPUT to SWC Takeover Relay Trigger
    9 = OUTPUT to Reverse Relay Trigger
    10 = OUTPUT to Power Relay Delayed Trigger
    11 = OUTPUT to RPi Shutdown Pin relay
    12 = OUTPUT to IHU Mute signal relay

  Future Ideas:
    Facilitate switching between RPi and other media PC system
      https://github.com/NicoHood/HID
      Use HID to enable switching the sent keys from OAP or Crankshaft
       letters to consumer/media keys via RawHID or serial command toggle?
      Use HID to send shutdown via System keyboard command
      store last used option to EEPROMex
      https://handsontec.com/index.php/arduino-5-storing-data-in-arduino-eeprom-memory/
    Use a pin to emulate GVIF mode button and switch to RGB input via
      detection of RGB SEL signal - if not possible via GVIF config already

*/
/**************************************************************************/

#include <Wire.h>
#include <Keyboard.h>
#include <Adafruit_MCP4725.h>

// ### dac constructor ### ///
Adafruit_MCP4725 dac;

// ### Constant Definitions ###
const int inDebugPin = 4;
const int inSWCPin = A0;
const int inRAPPin = 5;
const int inREVPin = 6;
const int inRGBPin = 7;
const int outSWCPin = 8;
const int outREVPin = 9;
const int outRAPPin = 10;
const int outSDWNPin = 11;
const int outMUTEPin = 12;
// For Adafruit MCP4725A1 the I2C address is 0x62 (default)
//  or 0x63 (A0 pin tied to VCC)
const int addrDAC = 0x62;

// ### Global Config Variables ###
// timer minutes for shutdown relay after loss of inRAPPin input
int delayPowerOff = 30;
// seconds offset for sending shutdown signal before shutdown relay timer expires
int delayShutdownOffset = 60;
// Number of loops to debounce input before changing input state
int debounceLoops = 10;
// millisecond threshold for long vs short SWC button press
int longThreshold = 1000;
// Number of buttons defined in the buttons array
int numSWCButtons = 0;
// +/-tolerance for SWC Trigger value
int SWCTolerance = 20;

// Global variables shared across functions
// Debug controls the serial output stream
bool flgDebug = false;
// Track the power status
bool flgPower = false;
// Track whether shutdown signal has been sent
bool flgShutdown = false;
// Track the reverse relay status
bool flgReverse = false;
// Track the RGB status
bool flgRGB = false;
// The 'active' SWC button
int activeSWC = 0;
// Record the function start time for reuse between functions
unsigned long loopStartTime = 0;

// running average Test
const int numSamplesSWC = 10;
static int readingsSWC[numSamplesSWC];
static int indexSWCReadings = 0;

// ### SWC Button ADC Config ###
// Define the available functions for SWC button actions
enum SWCFunctions {
  IDLE,     // The default SWC network state, should be 1st button defined!
  IGNORE,   // ignore this button and passthru to DAC withot intercepting
  PINOUT,   // control a pinout with this button
  USB,      // send a keyboard character with this button
  HOLDUSB,  // send a keyboard character but hold the button until released
  TOGGLEPIN // toggle the state of a pinout with this button
};
// Define the parameters for a single button
typedef struct SWCButton_t {
  char *name;              // Name of the button
  int trigger;             // the ADC value of the button
  uint32_t dac;            // The DAC value to send to IHU for button emulation
  SWCFunctions funcShort;  // Which function for a short press
  byte shortValue;         // What value for a short press
  SWCFunctions funcLong;   // Which function for a long press
  byte longValue;          // What value for a long press
};
// The DAC values are determined by measuring the SWC voltages using a
//  multi-meter (or ADC if you wish) from the IHU and calculated against
//  the Vref voltage (measure at 5V pin), also measured with a multi-meter.
//  The DAC values were calculated by this formula:
//   SWC voltage / (Vref / 4096) = DAC value (round to int value)
//  Using sensor shield 5V Vref = 5.014
//   4.414 / (5.014 / 4096) = 3605.85 = 3606
// These ADC values were taken from the IHU
//  For the A0 input, we use a resistor to create a voltage divider with
//  the steering wheel resistive ladder, the closest match to the IHU values
//  for the LR3 is 3.3k Ohms
const SWCButton_t SWCButtons[] = {
  {"IDLE", 908, 3632, IDLE, 0, IDLE, 0},
  {"MODE",295, 1180, IGNORE, 0, IGNORE, 0},
  {"VOL+", 599, 2396, IGNORE, 0, IGNORE, 0},
  {"VOL-", 95, 380, IGNORE, 0, IGNORE, 0},
  {"NEXT", 497, 1988, USB, 'N', HOLDUSB, 'N'},
  {"PREV", 396, 1584, USB, 'V', HOLDUSB, 'V'},
  {"ANS_CALL", 702, 2808, USB, 'P', USB, 'B'},
  {"END_CALL", 195, 780, USB, 'O', TOGGLEPIN, outREVPin},
  {"VOICE", 804, 3216, USB, 'M', TOGGLEPIN, outMUTEPin},
};

void setup() {
  /**************************************************************************/
  /*
    Function: setup
    Parameters: none
    Return: none

    Initial microcontroller setup is done here
  */
  /**************************************************************************/
  int loop;

  // Pull the input pins HIGH to avoid floating voltage issues
  pinMode(inDebugPin, INPUT_PULLUP);
  pinMode(inRAPPin, INPUT_PULLUP);
  pinMode(inREVPin, INPUT_PULLUP);
  pinMode(inRGBPin, INPUT_PULLUP);
  // Define the output pins
  pinMode(outSWCPin, OUTPUT);
  pinMode(outSDWNPin, OUTPUT);
  pinMode(outREVPin, OUTPUT);
  pinMode(outRAPPin, OUTPUT);
  pinMode(outMUTEPin, OUTPUT);

  // Check for debug
  if (!digitalRead(inDebugPin)) {
    flgDebug = true;
  }
  else  {
    flgDebug = false;
  }

  // Calculate the number of elements in the button array instead of hard
  // coding it
  numSWCButtons = sizeof(SWCButtons) / sizeof(SWCButton_t);
  // The CDC port speed
  Serial.begin(115200);
  // If debugging pin is grounded, wait for serial connection before
  //  completing setup
  if (flgDebug) {
    while (!Serial);
    Serial.println(F("Starting Vehicle Signal Microcontroller..."));
    Serial.print(F("Buttons Defined: "));
    Serial.println(numSWCButtons);
    Serial.println(F("Name\t\tTrigger\tshortFunc\tlongFunc"));
    for (loop = 0; loop < numSWCButtons; loop++) {
      Serial.print(SWCButtons[loop].name);
      Serial.print("\t\t");
      Serial.print(SWCButtons[loop].trigger);
      Serial.print("\t\t");
      Serial.print(SWCButtons[loop].funcShort);
      Serial.print("-");
      Serial.print(SWCButtons[loop].shortValue);
      Serial.print("\t\t\t");
      Serial.print(SWCButtons[loop].funcLong);
      Serial.print("-");
      Serial.println(SWCButtons[loop].longValue);
    }
  }

  // Initialize the keyboard interface
  Keyboard.begin();

  // Initialize the SWC DAC
  dac.begin(addrDAC);

  // Initialize the smoothing array to avoid hitting any button values during
  //  early running time
  for (indexSWCReadings = 0; indexSWCReadings < numSamplesSWC; indexSWCReadings++) {
    readingsSWC[indexSWCReadings] = analogRead(inSWCPin);
    // Allow the SWC value to stabilize by a short delay - during startup
    // is the only place delay will be used
    delay(10);
  }
  indexSWCReadings = 0;
}

// Function to monitorPowerState
void monitorPowerState() {
  /**************************************************************************/
  /*
    Function: monitorPowerState
    Parameters: none
    Return: none

    Monitor the inRAPPin state and manage the outRAPPin and outSDWNPin signals

    Shares state information via global variables flgPower and flgShutdown
      flgPower means the power relay is active
      flgShutdown means that the shutdown signal has been sent via outSDWNPin

    If inRAPPin goes LOW enable the power relay via outRAPPin and reset
    any timer that was in progress.

    If inRAPPin goes HIGH start a shutdown timer for delayShutdownOffset
    seconds, send a shutdown signal via outSDWNPin at delayShutdownOffset
    seconds before the relay is turned off
  */
  /**************************************************************************/
  // Timer for shutdown relay
  static unsigned long shutdownTime = 0;
  unsigned long delayPowerOffMillis;
  unsigned long delayPowerOffOffsetMillis;
  static int counterRAP = 0;
  static bool lastRAP = false;
  bool valueRAP;

  // Convenient shortcuts for the millis values needed
  delayPowerOffMillis = (unsigned long)delayPowerOff * 60 * 1000;
  delayPowerOffOffsetMillis = (unsigned long)delayShutdownOffset * 1000;

  // Check the Accessory Power signal to handle delayed shutdown
  // IMPORTANT NOTE: The pin status is inverted due to the optocoupler
  valueRAP = !digitalRead(inRAPPin);

  if ((valueRAP == lastRAP) && (counterRAP > 0)) {
    counterRAP--;
  }
  if (valueRAP != lastRAP) {
    counterRAP++;
  }
  if (counterRAP >= debounceLoops) {
    counterRAP = 0;
    lastRAP = valueRAP;
    if (!flgPower && valueRAP) {
      Serial.println(F("Accessory Power is detected, enabling power relay"));
      digitalWrite(outRAPPin, HIGH);
      flgPower = true;
      if (shutdownTime > 0) {
        shutdownTime = 0;
      }
    } else if (flgPower && shutdownTime > 0 && valueRAP) {
      Serial.println(F("Accessory Power is detected, cancelling shutdown"));
      shutdownTime = 0;
      flgShutdown = false;
    } else if (flgPower && shutdownTime == 0 && !valueRAP) {
      shutdownTime = delayPowerOffMillis + loopStartTime; 
      Serial.print(F("Accessory Power lost, starting shutdown delay timer for "));
      Serial.print((shutdownTime - loopStartTime) / 1000);
      Serial.println(F(" seconds"));

    }
  }
  if (flgPower && !valueRAP) {
    if (shutdownTime > 0) {
      if (loopStartTime % 10000 == 0) {
        Serial.print(F("Shutdown in ("));
        Serial.print((shutdownTime - loopStartTime) / 1000);
        Serial.println(F(") seconds!"));
      }
      if (loopStartTime >= shutdownTime) {
        Serial.println(F("Shutting Down!"));
        shutdownTime = 0;
        flgPower = false;
        flgShutdown = false;
        digitalWrite(outRAPPin, LOW);
      } else if (!flgShutdown
                  && loopStartTime >= shutdownTime - delayPowerOffOffsetMillis) {
        Serial.println(F("Sending shutdown signal"));
        digitalWrite(outSDWNPin, HIGH);
        flgShutdown = true;
      }
    }
  }
  // if debugging is enabled report the power status every 10 seconds
  if (flgDebug && loopStartTime % 10000 == 0) {
    Serial.print(F("Power Status="));
    Serial.print(flgPower);
    Serial.print(F(", Shutdown Status="));
    Serial.print(flgShutdown);
    if (shutdownTime > 0) {
        Serial.print(F(", shutdownTimer="));
        Serial.print((shutdownTime - loopStartTime) / 1000);
        Serial.print(F(" seconds"));
      }
    Serial.print(F(", inRAPPin Status="));
    Serial.println(valueRAP);
  }
}

void monitorReverseState() {
  /**************************************************************************/
  /*
    Function: monitorReverseState
    Parameters: none
    Return: none

    Monitor the inREVPin state and manage the outREVPin signal.

    inREVPin is debounced to prevent false triggers.

    Shares state information via global variable flgReverse
      flgReverse means that the reverse signal is active

    If inREVPin goes LOW enable the reverse relay via outREVPin.

    If inRAPPin goes HIGH disable the reverse relay via outREVPin.
  */
  /**************************************************************************/
  bool valueREV;
  static int counterREV = 0;

  // Check the Reverse signal to handle switching to the rear GVIF camera
  // IMPORTANT NOTE: The pin status is inverted due to the optocoupler
  valueREV = !digitalRead(inREVPin);

  if ((valueREV == flgReverse) && (counterREV > 0)) {
    counterREV--;
  }
  if (valueREV != flgReverse) {
    counterREV++;
  }
  if (counterREV >= debounceLoops) {
    flgReverse = valueREV;
    counterREV = 0;
    if (flgReverse) {
      Serial.println(F("Reverse Signal detected, enabling reverse relay"));
      digitalWrite(outREVPin, HIGH);
    } else {
      Serial.println(F("Reverse Signal lost, disabling reverse relay"));
      digitalWrite(outREVPin, LOW);
    }
  }
  if (flgDebug && loopStartTime % 10000 == 0) {
    Serial.print(F("Reverse Input Status="));
    Serial.print(flgReverse);
    Serial.print(F(", Reverse Relay="));
    Serial.println(digitalRead(outREVPin));
  }

}

void monitorRGBState() {
  /**************************************************************************/
  /*
    Function: monitorRGBState
    Parameters: none
    Return: none

    Monitor the inRGBPin state and help manage the SWC signal routing.

    inRGBPin is debounced to prevent false triggers.

    Shares state information via global variable flgRGB
      flgRGB means that the GVIF RGB selected signal is active

    We also toggle control of the SWC network based on the RGZB signal
    due to we can't read the state of the IHU to tell if the active input
    is AUX or not, so RGB serves as a crude proxy for that state.
  */
  /**************************************************************************/
  bool valueRGB;
  static int counterRGB = 0;

  // Check the RGB SEL signal to handle switching the SWC button routing
  // IMPORTANT NOTE: The pin status is inverted due to the optocoupler
  valueRGB = !digitalRead(inRGBPin);

  if ((valueRGB == flgRGB) && (counterRGB > 0)) {
    counterRGB--;
  }
  if (valueRGB != flgRGB) {
    counterRGB++;
  }
  if (counterRGB >= debounceLoops) {
    flgRGB = valueRGB;
    counterRGB = 0;
    if (flgRGB) {
      Serial.println(F("RGB is active GVIF output, taking over SWC"));
      digitalWrite(outSWCPin,HIGH);
    } else {
      Serial.println(F("RGB is not active GVIF output, releasing SWC"));
      digitalWrite(outSWCPin,LOW);
    }
  }
  if (flgDebug && loopStartTime % 10000 == 0) {
    Serial.print(F("RGB Selected Status="));
    Serial.println(flgRGB);
  }
}

// Function to sendButton
void sendButton(SWCFunctions action, byte value) {
  /**************************************************************************/
  /*
    Function: sendButton
    Parameters:
      action: The type of action to perform
      value: The value to use for the action
    Return: none

    For the requested action and passes the given value.  If flgDebug is set
    will not actually perform the USB action but will print status. This is
    to prevent stray characters from being accepted while debugging.

    Keyboard actions are only performed when flgRGB is true, GPIO actions are
    always performed.
  */
  /**************************************************************************/
  static bool flgKey = false;
  static bool flgPin = false;
  static byte heldValue = 0;

  // longPress actions are held until a new button state triggers
  if (flgKey) {
    Serial.print(F("Releasing held key '"));
    Serial.print(char(heldValue));
    Serial.println(F("'"));
    if (!flgDebug) {
      Keyboard.release(heldValue);
    }
    flgKey = false;
    heldValue = 0;
  }
  if (flgPin) {
    Serial.print(F("Releasing held GPIO pin '"));
    Serial.print(heldValue);
    Serial.println(F("'"));
    digitalWrite(heldValue, LOW);
    flgPin = false;
    heldValue = 0 ;
  }

  switch (action) {
    // Do nothing for these cases
    case IDLE:
    case IGNORE:
      break;
    case USB:
      if (flgRGB) {
        Serial.print(F("Sending Keyboard button '"));
        Serial.print(char(value));
        Serial.println(F("'"));
        if (!flgDebug) {
          Keyboard.write(value);
        }
      } else {
        Serial.print(F("USB Action '"));
        Serial.print(char(value));
        Serial.println(F("' ignored, not in RGB mode"));
      }
      break;
    case HOLDUSB:
      if (flgRGB) {
        Serial.print(F("Holding Keyboard button '"));
        Serial.print(char(value));
        Serial.println(F("'"));
        heldValue = value;
        flgKey = true;
        if (!flgDebug) {
          Keyboard.press(value);
        }
      } else {
        Serial.print(F("HOLDUSB Action '"));
        Serial.print(char(value));
        Serial.println(F("' ignored, not in RGB mode"));
      }
      break;
    case PINOUT:
      Serial.print(F("Holding GPIO Pin '"));
      Serial.print(value);
      Serial.println(F("'"));
      heldValue = value;
      flgPin = true;
      digitalWrite(value, HIGH);
      break;
    case TOGGLEPIN:
      // Note this does not set the global flags of the pin to avoid
      //  subverting the state change logic for externally triggered
      //  states - the external triggers should still work correctly
      Serial.print(F("Toggling GPIO Pin '"));
      Serial.print(value);
      Serial.println(F("'"));
      // This may only work like this on AVR based arduinos, other boards
      //  may require a more complex method to toggle the pin state
      digitalWrite(value, !digitalRead(value));
      break;
  }
}

void monitorSWCInState() {
/**************************************************************************/
/*
  Function: monitorSWCInState
  Parameters: none
  Return: none

  Read the inSWCPin value and translate to a button definition
  Handle button press events as appropriate
  Stores the active button in activeSWC global var

  Uses button change state as a trigger for starting/stopping button timmer
  which is used to track if button press is short or long.

  The ADC readings are smoothed over 10 samples, then that average is
  'debounced' over 10 iterations for button state change.  Why do both?
  In bench testing, the ADC readings were very stable using USB power, but
  far less stable when using external power regardless of the source.  Doing
  both prevents some of the transient button states I observed in bench
  testing.
*/
/**************************************************************************/
int loop;
// Log the time a button press change started
static unsigned long buttonTimer = 0;
// The current SWC reading
int valueSWC = 0;
// The detected SWC button for the current loop
int currentSWC = 0;
// The last debounced SWC button state
static int lastSWC = 0;
// number of loops the current button state has varied from lastSWC
static int counterSWC = 0;
// Does the valueSWC map to a defined button state?
bool foundButton = false;
// Has the pressed button changed?
bool buttonChanged = false;
// carry direct button pressed buttons forward
static bool buttonCarryForward = false;
// Has the long press event been sent?
static bool buttonLongPress = false;
int totalReadings = 0;

  readingsSWC[indexSWCReadings] = analogRead(inSWCPin);
  for (loop = 0; loop < numSamplesSWC; loop++) {
    totalReadings += readingsSWC[loop];
  }
  indexSWCReadings++;

  if (indexSWCReadings >= numSamplesSWC) {
    indexSWCReadings = 0;
  }

  //valueSWC = analogRead(inSWCPin);
  valueSWC = totalReadings / numSamplesSWC;
  
  // If debugging is enabled output the valueSWC every 1/4 second
  if (flgDebug && loopStartTime % 250 == 0) {
    Serial.print(F("inSWCPin Value: "));
    Serial.println(valueSWC);
  }

  // Check valueSWC against button Definitions
  for (loop = 0; loop < numSWCButtons; loop++) {
    if (valueSWC <= SWCButtons[loop].trigger - SWCTolerance
        || valueSWC >= SWCButtons[loop].trigger + SWCTolerance) {
      continue;
    } else {
      foundButton = true;
      currentSWC = loop;
    }
  }

  // debounce the button signal
  if (foundButton) {
    if ((currentSWC == lastSWC) && (counterSWC > 0)) {
      counterSWC--;
    }
    if (currentSWC != lastSWC) {
      counterSWC++;
    }
    if (counterSWC >= debounceLoops) {
      Serial.print(F("Changing active button from "));
      Serial.print(SWCButtons[lastSWC].name);
      Serial.print(F(" to "));
      Serial.println(SWCButtons[currentSWC].name);
      counterSWC = 0;
      lastSWC = currentSWC;
      buttonChanged = true;
    }
  }
  if (buttonChanged || buttonCarryForward) {
    if (buttonCarryForward) {
      buttonCarryForward = false;
    }
    // WAS idle changed to new button (or carried idle)
    if (activeSWC == 0) {
      if (flgDebug) {
        Serial.print(F("Starting buttonTimer for "));
        Serial.println(SWCButtons[currentSWC].name);
      }
      activeSWC = currentSWC;
      buttonTimer = loopStartTime;
    // Change from one button directly to another
    } else if ((currentSWC != 0) && (activeSWC != currentSWC)) {
      if (flgDebug) {
        Serial.print(F("activeSWC was "));
        Serial.print(SWCButtons[activeSWC].name);
        Serial.print(F(" but currentSWC is "));
        Serial.print(SWCButtons[currentSWC].name);
        Serial.print(F(" activeSWC was pressed for "));
        if (buttonTimer > 0) {
          Serial.println(loopStartTime - buttonTimer);
        } else {
          Serial.println(F("No Timer"));
        }
      }
      // Only send a keypress if there was a timer active
      if (buttonTimer > 0) {
        if (!buttonLongPress) {
          if (loopStartTime - buttonTimer >= longThreshold) {
            buttonLongPress = false;
            sendButton(SWCButtons[activeSWC].funcLong, SWCButtons[activeSWC].longValue);
          } else {
            sendButton(SWCButtons[activeSWC].funcShort, SWCButtons[activeSWC].shortValue);
          }
        }
      }
      // Reset the button timer and state for the current button
      activeSWC = IDLE;
      buttonTimer = 0;
      buttonCarryForward = true;
      buttonLongPress = false;
    } else {
      // returned to idle from button press
      if (flgDebug) {
        Serial.print(SWCButtons[activeSWC].name);
        Serial.print(F(" was pressed for "));
        Serial.println(loopStartTime - buttonTimer);
      }
      if (buttonLongPress) {
        buttonLongPress = false;
        activeSWC = IDLE;
        buttonTimer = 0;
        // Reset any held button, use 99 as a debug marker since value ignored
        sendButton(IDLE, 99);
      } else {
        if (loopStartTime - buttonTimer >= longThreshold) {
          buttonLongPress = true;
          sendButton(SWCButtons[activeSWC].funcLong, SWCButtons[activeSWC].longValue);
        } else {
          sendButton(SWCButtons[activeSWC].funcShort, SWCButtons[activeSWC].shortValue);
        }
        buttonLongPress = false;
        activeSWC = IDLE;
        buttonTimer = 0;
        sendButton(IDLE, 99);
      }
    }
  }
  // If a button is held down longer than longThreshold do the button action
  // regardless of button change state
  if (buttonTimer > 0) {
    if ((loopStartTime - buttonTimer >= longThreshold) && !buttonLongPress) {
      buttonLongPress = true;
      sendButton(SWCButtons[activeSWC].funcLong, SWCButtons[activeSWC].longValue);
    }
  }
  if (flgDebug && foundButton && (lastSWC != currentSWC) && counterSWC > 0) {
    Serial.print(F("Detected button press: "));
    Serial.print(currentSWC);
    Serial.print(" - '");
    Serial.print(SWCButtons[currentSWC].name);
    Serial.print("' (");
    Serial.print(counterSWC);
    Serial.println(F(") times"));
  }
  if (flgDebug && loopStartTime % 1000 == 0) {
    Serial.print(F("Active Button "));
    Serial.print(SWCButtons[activeSWC].name);
    Serial.print(F(" Timer Timestamp "));
    Serial.println(buttonTimer);
  }
}

void setSWCOutState() {
  /**************************************************************************/
  /*
    Function: setSWCOutState
    Parameters: none
    Return: none

    Set the outSWCPin value based on activeSWC and flgRGB, outSWCPin always
    follows the activeSWC button state.

    Outgoing SWC state is based strictly on the funcShort actions.  If the
    short function is IDLE or IGNORE those values are always passed through.

    If flgRGB is not active, the activeSWC is also passed through.

    If flgRGB is active, the idle state is sent instead so that the SWC interupts
    are only processed by the openauto system instead.
  */
  /**************************************************************************/
  uint32_t dacOutput;

  if (SWCButtons[activeSWC].funcShort == IDLE ||
      SWCButtons[activeSWC].funcShort == IGNORE ||
      !flgRGB) {
    dacOutput = SWCButtons[activeSWC].dac;
  } else {
    // Default to IDLE state
    dacOutput = SWCButtons[IDLE].dac;
  }
  dac.setVoltage(dacOutput, false);
  if (flgDebug && loopStartTime % 1000 == 0) {
    Serial.print("DAC Output value: ");
    Serial.println(dacOutput);
  }
}

void loop() {
  /**************************************************************************/
  /*
    Function: loop
    Parameters: none
    Return: none

    Monitors the incoming signals and generates appropriate outgoing responses

    Avoids blocking using delay calls, but is throttled to run a maximum of
    only once per millisecond to prevent serial buffer issues when debugging
  */
  /**************************************************************************/
  int loop;
  static unsigned long endTime = 0;

  loopStartTime = millis();

  // Make sure at least 1 millisecond has passed between loops
  if (loopStartTime > endTime) {
    // Check for debug status change
    if (!digitalRead(inDebugPin)) {
      flgDebug = true;
    }
    else  {
      flgDebug = false;
    }

    monitorPowerState();
    monitorReverseState();
    monitorRGBState();
    monitorSWCInState();
    setSWCOutState();

    endTime = millis();
  }
}
