/* Input voltage 15 - 22V
   Output voltage 1 - 14.4V
   Simple MPPT solar charge controller for 18V solar panels
   Sparkfun Pro Micro 5V, 16MHz or 3.3V, 8MHz (3.3v recommended, more efficient)
   ACS712 current sensor on the OUTPUT side
   Voltage dividers for voltage measurement on panel and output side
   Two N-channel mosfets, driven by IR2104 half bridge driver, inductor (synchronous buck converter)
   Supplied by the panel voltage, can't drain your battery during the night
   Working frequency 31.5kHz
   WARNING! This device is not intended to drive 5V USB devices directly!
   Always use a regulated 5V USB adapter on the output! Otherwise, voltage glichtes may damage your USB device!
   This controller is COMMON NEGATIVE!
   3 operation modes: MPPT, CV, CC
   SD card data logger for time, voltage and current. You can import the txt files in Excel
   WARNING! Always adjust output voltage and output current limits according to your battery type!!
   Efficiency between 84% and 92% (excluding board supply current of about 75mA)
*/

const float codeVersion = 1.0; // Software revision

//
// =======================================================================================================
// BUILD OPTIONS (comment out unneeded options)
// =======================================================================================================
//

//#define DEBUG // if not commented out, Serial.print() is active! For debugging only!!

//
// =======================================================================================================
// INCLUDE LIRBARIES
// =======================================================================================================
//

// Tabs (header files in sketch directory)
#include "readVCC.h"

// Libraries
#include <SPI.h>
#include <Wire.h>
#include <statusLED.h> // TheDIYGuy999 library: https://github.com/TheDIYGuy999/statusLED
#include <PWMFrequency.h> // https://github.com/TheDIYGuy999/PWMFrequency
#include <U8g2lib.h> // https://github.com/olikraus/u8g2
#include <SdFat.h>

//
// =======================================================================================================
// PIN ASSIGNMENTS & GLOBAL VARIABLES
// =======================================================================================================
//

// Status LED objects
#ifdef __AVR_ATmega32U4__ // Pro Micro Board
statusLED LED(true); // true = inversed (LED wired between VCC and output)
#else // Pro Mini Board
statusLED LED(false); // false = not inversed (LED wired between GND and output)
#endif

// SD card
SdFat SD;
File TimeFile;
File VoltFile;
File CurFile;

// OLED display SSD1306
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// output pins
#define PWM 9
#define ENABLE 8
#define VSENSE_IN A1
#define ISENSE_OUT A2
#define VSENSE_OUT A0
#define POT A3
#define CHIP_SELECT 10

// Global variables
unsigned long displayInterval = 1000;
unsigned long loggingInterval = 15000; // 15s
float inputVoltage;
float inputCurrent;
float outputPower;
float outputPowerDelta;
float outputPowerPrevious = 9999.0; // Init value above max. panel power!
float outputVoltage;
float outputVoltagePrevious;
float outputCurrent;
float energy;
boolean undervolt;
boolean SDpresent;
boolean displayOn = true;

byte controlMode;
#define CV 1 // Constant Voltage
#define CC 2 // Constant Current
#define MPPT 3 // Maximum Power Point Tracking
#define BP 4 // Battery protection mode

// ACS712 current sensor calibration variables
#if F_CPU == 16000000 // ACS712 5V supply: 
const float acs712VoltsPerAmp = 0.185; // 0.185 for 5A version, 0.100 for 20A, 0.066 for 30A
#endif
#if F_CPU == 8000000 // ACS712 3.3V supply (outside datasheet range...): 
const float acs712VoltsPerAmp = 0.1221; // 0.1221 for 5A version, 0.066 for 20A, 0.04356 for 30A
#endif
const int acs712Offset = 503; // Zero offset is usually 512 = 1/2 of ADC range 5V:509, 3.3V: 503

float pwm; // float required for finer granularity during calculadion in differential equations!
boolean trackingDownwards; // true = downwards
float vcc = 4.5; // Init value only. Will be read automatically later on

// Configuration variables
float minPanelVoltage = 12.0; // 12.0
float targetPanelVoltage = 14.0; // 14.0 (calculated by MPPT algorithm)
float maxPanelVoltage = 16.0; // 16.0
// !!CAUTION: targetOutputVoltage is adjusted with the potentiometer!!
float targetOutputVoltage = 4.2; // init value only! see above!!
float trackingIncrement = 0.5; // MPPT tracking voltage step 0.5V
float maxOutputCurrent = 3.0; // the desired output current limit. 3.0A requires heat sink on diode!!
float efficiency = 0.84; // About 84% (required for input current calculation)

// Buttons
// macro for detection of falling edge and debouncing
/*the state argument (which must be a variable) records the current
  and the last 7 reads by shifting one bit to the left at each read.
  If the value is 240(=0b11110000) we have one falling edge followed by
  4 consecutive 0's. That would qualify as a debounced falling edge*/
#define DFE(signal, state) (state=(state<<1)|signal)==B11110000

// Falling state variables for each button
byte buttonMenuFallingState;
byte buttonPlusFallingState;
byte buttonMinusFallingState;
byte buttonBackFallingState;

// Button pins
#define BUTTON_MENU 4
#define BUTTON_MINUS 5
#define BUTTON_PLUS 6
#define BUTTON_BACK 7

//
// =======================================================================================================
// MAIN ARDUINO SETUP (1x during startup)
// =======================================================================================================
//

void setup() {

  // Serial port setup
#ifdef DEBUG
  Serial.begin(19200);
#endif

  // SD card setup
  SD.begin(CHIP_SELECT);

  // PWM frequency
  setPWMPrescaler(PWM, 1 );  // 1 = 31.5kHz

  // Output setup
  pinMode(PWM, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  // Button setup
  pinMode(BUTTON_MENU, INPUT_PULLUP);
  pinMode(BUTTON_MINUS, INPUT_PULLUP);
  pinMode(BUTTON_PLUS, INPUT_PULLUP);
  pinMode(BUTTON_BACK, INPUT_PULLUP);

  // LED setup
#ifdef __AVR_ATmega32U4__ // Pro Micro Board
  LED.begin(17); // Onboard LED on pin 17
#else // Pro Mini Board
  LED.begin(13); // Onboard LED on pin 17
#endif

  // Display setup
  u8g2.begin();
  u8g2.setFontRefHeightExtendedText();
  u8g2.setFont(u8g_font_7x14);

  // switch off output
  analogWrite(PWM, 12); // not 0 to keep bootstrap circuit running!
}

//
// =======================================================================================================
// READ POTENTIOMETER (lets you select the voltage range you want)
// =======================================================================================================
//
void readPot() {
  targetOutputVoltage = analogRead(POT) / 1023.0 + 4.2; // 4.2 - 5.2V
  //targetOutputVoltage = analogRead(POT) / 71.04; // 0 - 14.4V
}

//
// =======================================================================================================
// READ BUTTONS
// =======================================================================================================
//

boolean readButtons() {

  // Display stays on for 10s, after a button was pressed
  static unsigned long displayDelay;
  if (millis() - displayDelay >= 10000) {
    displayOn = false;
  }

  // Read button states every 5 ms (debounce time):
  static unsigned long lastDebounce;
  if (millis() - lastDebounce >= 5) {
    lastDebounce = millis();

    // Menu button
    if (DFE(digitalRead(BUTTON_MENU), buttonMenuFallingState)) {
      displayOn = true;
      displayDelay = millis();
    }

    // Minus button
    if (DFE(digitalRead(BUTTON_MINUS), buttonMinusFallingState)) {
      displayOn = true;
      displayDelay = millis();
    }

    // Plus button
    if (DFE(digitalRead(BUTTON_PLUS), buttonPlusFallingState)) {
      displayOn = true;
      displayDelay = millis();
    }

    // Back button
    if (DFE(digitalRead(BUTTON_BACK), buttonBackFallingState)) {
      displayOn = true;
      displayDelay = millis();
    }

  }
}

//
// =======================================================================================================
// READ SENSORS
// =======================================================================================================
//

// Averaging subfunctions
float averageA() { // Input power (running average)
  static float raw[4];

  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  raw[0] = (analogRead(ISENSE_OUT) - acs712Offset) * vcc / acs712VoltsPerAmp / 1024;
  float average = (raw[0] + raw[1] + raw[2] + raw[3]) / 4.0;
  return average;
}


// Main sensor read function
void readSensors() {

  inputVoltage = analogRead(VSENSE_IN) * vcc / 93; // 1023 = vcc * 110 / 10 = 1023 / 55 = 18.6

  outputCurrent = averageA();

  outputPower = outputVoltage * outputCurrent;

  outputVoltage = analogRead(VSENSE_OUT) * vcc / 92; // 1023 = vcc * 110 / 10 = 1023 / 55 = 18.6

  inputCurrent = outputCurrent * outputVoltage / inputVoltage / efficiency;
}

//
// =======================================================================================================
// LOCKOUT SUB FUNCTION
// =======================================================================================================
//

void lockout() {
  pwm = 12; // not 0 to keep bootstrap circuit running!
  //digitalWrite(ENABLE, LOW); // Disable Mosfet driver
  analogWrite(PWM, pwm); // Switch output off
#ifdef DEBUG
  Serial.print("Over voltage lockout! ");
  Serial.println(outputVoltage);
#endif
}

//
// =======================================================================================================
// MPPT TRACKING
// =======================================================================================================
//
void mppt() {

  /* MPPT Strategy:
      There are tree controllers:
      - Output voltage controller
      - Output current controller
      - MPPT tracker, if output voltage is below target
  */

  // Read current voltages and current
  readSensors();

  // Panel undervoltage lockout ---------------------------------------------------------------------------
  while (outputVoltage < 1.0 && inputVoltage < 15.0) {  // 1.0 15.0
    pwm = 70;
    digitalWrite(ENABLE, LOW); // Disable Mosfet driver
    analogWrite(PWM, pwm);
    LED.on();
    Serial.println("Panel undervoltage, waiting for more sun...");
    undervolt = true;
    delay(2000);
    checkVcc();
    readSensors();
    drawDisplay();
  }

  // Voltage and current controllers -----------------------------------------------------------------------

  // If output voltage is near desired voltage: control target = output voltage! ---
  if (outputVoltage > (targetOutputVoltage - 0.2) && (outputCurrent <= maxOutputCurrent) ) {
    pwm += targetOutputVoltage - outputVoltage; // simple p (differential) controller
    if (inputVoltage < minPanelVoltage) pwm -= minPanelVoltage - inputVoltage;
    controlMode = CV; // Constant Voltage Mode
  }

  // Else if current only is above limit: control target = output current! ---
  else if ((outputCurrent > maxOutputCurrent) && (outputVoltage < targetOutputVoltage)) {
    pwm -= outputCurrent - maxOutputCurrent;
    controlMode = CC; // Constant Current Mode
  }

  // Else if current AND voltage are above limit: decrease PWM ---
  else if ((outputCurrent > maxOutputCurrent) && (outputVoltage > targetOutputVoltage)) {
    pwm --;
    controlMode = BP; // Battery Protection Mode
  }

  // else: control target = MPPT ---
  else {
    controlMode = MPPT; // Maximum Power Point Tracking

    // MPPT (max. input power) tracking direction (upwards / downwards is related to panel voltage!)
    static unsigned long lastMppt;
    if (millis() - lastMppt >= 1000) { // Every 1000ms
      lastMppt = millis();

      // Calculate power delta
      outputPowerDelta = outputPower - outputPowerPrevious;

      if (trackingDownwards) targetPanelVoltage -= trackingIncrement;
      else targetPanelVoltage += trackingIncrement;


      // Tracking direction is depending on the panel voltage, if outside limits!
      if (targetPanelVoltage <= minPanelVoltage) {
        targetPanelVoltage = (minPanelVoltage + maxPanelVoltage) / 2;
#ifdef DEBUG
        Serial.println("PV lim. -");
#endif
      }
      else if (targetPanelVoltage >= maxPanelVoltage) {
        targetPanelVoltage = (minPanelVoltage + maxPanelVoltage) / 2;
#ifdef DEBUG
        Serial.println("PV lim. +");
#endif
      }

      else { // if within voltage limits, search for maximum power point!
        // Wrong tracking direction (less power than previously), so change it!
        if (outputPowerDelta < 0.1) { // 0.03A current sensor step * 20V = 0.6W
          trackingDownwards = !trackingDownwards;
        }
      }

      // Store previous power for next comparison
      outputPowerPrevious = outputPower;
    }

    // Calculate deviation
    static unsigned long lastCalc;
    if (millis() - lastCalc >= 50) { // Every 50ms (prevent it from oscillating in low light condidions)
      lastCalc = millis();
      pwm -= targetPanelVoltage - inputVoltage; // simple p (differential) controller
    }
  }

  // Protection ----------------------------------------------------------------------------------------
  if (outputVoltage > (targetOutputVoltage + 1.0)) lockout(); // Output overvoltage protection

  // Write PWM output ----------------------------------------------------------------------------------
  pwm = constrain(pwm, 12, 242 ); // 5 - 95%, because of bootstrap circuit!
  //pwm = analogRead(POT) * 256 / 1024;
  serialPrint();

  undervolt = false;
  digitalWrite(ENABLE, HIGH); // Enable Mosfet driver
  analogWrite(PWM, pwm);
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//
void led() {
  if (!displayOn) {
    if (controlMode == MPPT) {
      // Indicate panel voltage: 14 flashes = 14V etc.
      LED.flash(20, 380, 700, inputVoltage); // ON, OFF, PAUSE, PULSES
    }
    if (controlMode == CV) {
      //Constant voltage mode (flickering)
      LED.flash(30, 100, 0, 0);
    }
    if (controlMode == CC) {
      //Constant current mode (fast flickering)
      LED.flash(30, 50, 0, 0);
    }
    if (controlMode == BP) {
      //Battery protection mode (very fast flickering)
      LED.flash(30, 25, 0, 0);
    }
  }
  else LED.off();
}

//
// =======================================================================================================
// CHECK VCC VOLTAGE
// =======================================================================================================
//
void checkVcc() {

  static unsigned long lastVcc;
  if (millis() - lastVcc >= 1000) { // Every 1000ms
    lastVcc = millis();
    vcc = readVcc() / 1000.0;
  }
}

//
// =======================================================================================================
// SERIAL PRINT
// =======================================================================================================
//
void serialPrint() {
#ifdef DEBUG
  static unsigned long lastPrint;
  if (millis() - lastPrint >= 1000) { // Every 1000ms
    lastPrint = millis();

    // Mode
    if (controlMode == MPPT) {
      Serial.print("MPPT ");
      Serial.print(trackingDownwards);
    }
    if (controlMode == CV) {
      Serial.print("CV   ");
    }

    if (controlMode == CC) {
      Serial.print("CC   ");
    }

    if (controlMode == BP) {
      Serial.print("BP   ");
    }

    // Input
    Serial.print("\t In T. V: ");
    Serial.print(targetPanelVoltage);
    Serial.print("\t In V: ");
    Serial.print(inputVoltage);
    Serial.print("\t A: ");
    Serial.print(inputCurrent);


    // Output
    Serial.print("\t Out T. V: ");
    Serial.print(targetOutputVoltage);
    Serial.print("\t Out V: ");
    Serial.print(outputVoltage);
    Serial.print("\t A: ");
    Serial.print(outputCurrent);
    Serial.print("\t W: ");
    Serial.print(outputPower);
    Serial.print("\t Delta W: ");
    Serial.print(outputPowerDelta);
    Serial.print("\t PWM: ");
    Serial.print(pwm);
    Serial.print("\t vcc: ");
    Serial.println(vcc);
  }
#endif
}

//
// =======================================================================================================
// DISPLAY LOOP
// =======================================================================================================
//

void drawDisplay() {
  static unsigned long lastDisplay;
  if (millis() - lastDisplay >= displayInterval) {
    lastDisplay = millis();

    // Do energy calculation
    energy = energy + outputVoltage * outputCurrent / (3600 * (1000 / displayInterval)); // Wh / 3600 = Ws

    u8g2.firstPage();  // clear screen
    do {
      if (displayOn) {
        u8g2.setCursor(0, 16);
        u8g2.print("V:  ");
        u8g2.print(outputVoltage);

        u8g2.setCursor(79, 16);
        u8g2.print(inputVoltage);

        u8g2.setCursor(0, 32);
        u8g2.print("A:  ");
        u8g2.print(outputCurrent, 3);

        u8g2.setCursor(0, 48);
        u8g2.print("W:  ");
        u8g2.print(outputVoltage * outputCurrent);

        u8g2.setCursor(0, 64);
        u8g2.print("Wh: ");
        u8g2.print(energy, 3);

        // Mode & messages
        u8g2.setCursor(79, 64);
        if (undervolt) {
          u8g2.print("INP. <");
        }
        else {

          if (controlMode == MPPT) u8g2.print("MPPT ");
          if (controlMode == CV) u8g2.print("CV");
          if (controlMode == CC) u8g2.print("CC");
          if (controlMode == BP) u8g2.print("BP");
        }

        // SD card presence
        u8g2.setCursor(79, 48);
        if (SDpresent) u8g2.print("SD OK");
        else u8g2.print("No SD");
      }
    } while ( u8g2.nextPage() ); // show display queue
  }
}

//
// =======================================================================================================
// WRITE SD CARD
// =======================================================================================================
//

void writeSD() {
  static unsigned long lastLog;
  if (millis() - lastLog >= loggingInterval) {
    lastLog = millis();

    TimeFile = SD.open("TIME.txt", FILE_WRITE);
    if (TimeFile) {
      TimeFile.println(millis() / 1000); // Time in s
      TimeFile.close();
      SDpresent = true;
    }
    else SDpresent = false;

    VoltFile = SD.open("VOLT.txt", FILE_WRITE);
    if (VoltFile) {
      VoltFile.println(outputVoltage);
      VoltFile.close();
    }

    CurFile = SD.open("CUR.txt", FILE_WRITE);
    if (CurFile) {
      CurFile.println(outputCurrent, 3);
      CurFile.close();
    }
  }
}

//
// =======================================================================================================
// MAIN LOOP
// =======================================================================================================
//

void loop() {
  readPot();
  readButtons();
  checkVcc();
  mppt();
  led();
  drawDisplay();
  writeSD();
}
