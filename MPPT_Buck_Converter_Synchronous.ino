/* Input voltage 12 - 22V
   Output voltage 2.5 - 14.4V
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
   4 operation modes: MPPT, CV, CC, IDLE
   SD card data logger for time, voltage and current. You can import the txt files in Excel
   WARNING! Always adjust output voltage and output current limits according to your battery type!!
   Efficiency between 77% and 95% (including board supply current of about 75mA)
   Anti backfeed protection (MOSFET Q1)
*/

const float codeVersion = 1.2; // Software revision

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
#include <U8g2lib.h> // https://github.com/olikraus/u8g2 IMPORTANT: use 2.0.7, newer versions require too much flash menory!
//#include <U8x8lib.h>
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
//U8X8_SSD1306_128X64_NONAME_HW_I2C u8g2;

// output pins
#define PWM 9
#define ENABLE 8
#define VSENSE_IN A1
#define ISENSE_OUT A2
#define VSENSE_OUT A0
#define POT A3
#define CHIP_SELECT 10

// Global variables
unsigned long displayInterval = 500;
unsigned long loggingInterval = 15000; // 15s
float inputVoltage;
float inputCurrent;
float outputPower;
float outputPowerDelta;
float outputPowerPrevious = 9999.0; // Init value above max. panel power!
float outputVoltage;
float outputVoltageDelta;
float outputVoltagePrevious;
float outputCurrent;
float energy;
int pwmMin = 30; // 30, 12% (equals to 2.5V minimum output voltage @ 21V supply voltage)
const int pwmMax = 242; // 242, 95% never more, because of charge pump operation!
boolean buckIdle;
boolean SDpresent;
boolean displayOn = true;
boolean flagNegativePower = false;

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
const int acs712Offset = 502; // Zero offset is usually 512 = 1/2 of ADC range 5V:509, 3.3V: 502 (adjust it, until amps show zero)

float pwm; // float required for finer granularity during calculation in differential equations!
boolean trackingDownwards; // true = downwards
float vcc = 3.3; // Init value only. Will be read automatically later on. Base for all our voltage readings!

// Configuration variables
const float minPanelVoltage = 12.0; // 12.0
float targetPanelVoltage = 14.0; // 14.0 (calculated by MPPT algorithm)
float maxPanelVoltage = 17.9; // was 16.0, should be Vpm (about 17.9) of your panel
float panelOverVoltage = 22.0;
float lowPowerThreshold = 3.0; // 3 Watt (for max panel voltage switching)
// !!CAUTION: targetOutputVoltage is adjusted with the potentiometer!!
float targetOutputVoltage = 4.2; // init value only! see above!!
float trackingIncrement = 0.5; // MPPT tracking voltage step (min. 0.5V, better 0.75, otherwise tracking will not work, because power delta is too small)
float trackingDirectionChangeWattsThreshold = 0.1; // Tracking direction will change, if output power delta is below or even negative (about 0, up to 0.1)
float maxOutputCurrent = 4.0; // the desired output current limit. (your hardware limit, the inductor im my case)
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
  LED.begin(13); // Onboard LED on pin 13
#endif

  // Display setup
  u8g2.begin();
  u8g2.setFontRefHeightExtendedText();
  u8g2.setFont(u8g_font_7x14);
  drawDisplay();

  // Check vcc voltage (base for all voltage readings!)
  checkVcc(true); // true = do it immeadiately

  // Read sensors
  delay(2000); // let voltages settling down
  readSensors();
  drawDisplay();

  // Read Potentiometer
  readPot();

  // Idle buck converter (low side MOSFET protection)
  buckConverterIdle();

  // SD card setup
  SD.begin(CHIP_SELECT);
  writeSD(true); // do first log without delay (true)!
}

//
// =======================================================================================================
// READ POTENTIOMETER (select the voltage range you want)
// =======================================================================================================
//
void readPot() {
  //targetOutputVoltage = analogRead(POT) / 1023.0 + 4.2; // 4.2 - 5.2V
  //targetOutputVoltage = analogRead(POT) / 71.04; // 0 - 14.4V
  //targetOutputVoltage = analogRead(POT) / 100.294 + 4.2; // 4.2 - 14.4V (USB charger & 3S lead acid) <--
  //targetOutputVoltage = analogRead(POT) / 132.0 + 4.75; // 4.75 - 12.5V (USB charger & 3S LiPo)
}

//
// =======================================================================================================
// READ BUTTONS
// =======================================================================================================
//

boolean readButtons() {

  // Display stays on for 60s, after a button was pressed
  static unsigned long displayDelay;
  if (millis() - displayDelay >= 60000) {
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

static unsigned long delayNegativePower;

// Averaging subfunctions
float averageOutputA() { // Output power (running average)
  static float raw[4];

  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  raw[0] = (analogRead(ISENSE_OUT) - acs712Offset) * vcc / acs712VoltsPerAmp / 1024;
  float average = (raw[0] + raw[1] + raw[2] + raw[3]) / 4.0;
  return average;
}

float averageOutputVoltage() { // Output voltage (running average)
  static float raw[4];

  raw[3] = raw[2];
  raw[2] = raw[1];
  raw[1] = raw[0];
  raw[0] = analogRead(VSENSE_OUT) * vcc / 93; // 110k / 10k = 11. 1023 / 11 = 93
  float average = (raw[0] + raw[1] + raw[2] + raw[3]) / 4.0;
  return average;
}

// Main sensor read function
void readSensors() {

  outputVoltage = averageOutputVoltage();
  outputCurrent = averageOutputA();
  outputPower = outputVoltage * outputCurrent;

  inputVoltage = analogRead(VSENSE_IN) * vcc / 93; // 110k / 10k = 11. 1023 / 11 = 93
  inputCurrent = outputCurrent * outputVoltage / inputVoltage / efficiency;

  // Low side mosfet protection (preventing pwm from going too low and shorting battery to GND)
  if (controlMode == CV) pwmMin = (255 * targetOutputVoltage / inputVoltage) - 2; // Constant Voltage mode
  else if (controlMode == MPPT) pwmMin = (255 * outputVoltage / inputVoltage) - 2; // MPPT mode
  else pwmMin = 30;

  pwmMin = constrain(pwmMin, 30, pwmMax); // Never allow pwm below 30

  // Negative power flag (battery drain protection during night)
  /*if (outputPower >= -0.3) delayNegativePower = millis();
    if (millis() - delayNegativePower > 100) flagNegativePower = true;
    else flagNegativePower = false;*/
  if (outputPower >= -0.3) flagNegativePower = false;
  else flagNegativePower = true;
}

//
// =======================================================================================================
// BUCK CONVERTER IDLE FUNCTION (IR2104 disabled, but PWM synchronous with measured voltage levels)
// =======================================================================================================
//

void buckConverterIdle() {
  buckIdle = true;
  digitalWrite(ENABLE, LOW); // Disable Mosfet driver (current can't flow from battery thru low side FET to GND)
  pwm = 255 * outputVoltage / inputVoltage; // Keep the PWM signal synchronous to the voltage levels
  pwm = constrain(pwm, pwmMin, pwmMax ); // 5 - 95%, because of bootstrap circuit!
  analogWrite(PWM, pwm);
#ifdef DEBUG
  Serial.print("Buck Converter Idle!");
  Serial.println(outputVoltage);
#endif
}

//
// =======================================================================================================
// BUCK CONVERTER DRIVE FUNCTION (IR2104 enabled, PWM according to required voltages)
// =======================================================================================================
//

void buckConverterDrive() {
  buckIdle = false;
  pwm = constrain(pwm, pwmMin, pwmMax ); // 5 - 95%, because of bootstrap circuit!
  analogWrite(PWM, pwm);
  digitalWrite(ENABLE, HIGH); // Enable Mosfet driver
#ifdef DEBUG
  serialPrint();
#endif
}

//
// =======================================================================================================
// MPPT TRACKING
// =======================================================================================================
//
void mppt() {

  /* MPPT Strategy:
      There are three controllers:
      - Output voltage controller
      - Output current controller
      - MPPT tracker, if output voltage is below target
  */

  static unsigned long lastMppt;
  static unsigned long delayHighMaxPanelVoltage;

  static bool cv;
  static bool cc;
  static bool bp;

  // Read current voltages and current
  readSensors();

  // Panel undervoltage & negative current lockout (prevents from current flowing backwards)----------------
  //while ((outputVoltage < 1.0 && inputVoltage < 15.0) // while output < 1.0V and input < 15.0V // only working with anti feedback protection, for example with TP4056!
  while (inputVoltage < minPanelVoltage - 1.0 // while input voltage < about 11.0V (sunset condition)
         || flagNegativePower // or negative power (sunset condition)
         || inputVoltage > panelOverVoltage // or panel voltage is pushed too high from battery (wrong pwm)
         || outputVoltage > inputVoltage // or output V > input V
        ) {
    buckConverterIdle(); // disable MOSFET driver, but keep pwm synchronuos with voltages
    checkVcc(false); // with delay
    lastMppt = millis();
    readPot();
    readSensors();
    readButtons();
    drawDisplay();
    led();
  }

  // Voltage and current controllers -----------------------------------------------------------------------

  // If output voltage is above desired voltage: control target = output voltage! ---
  if (outputVoltage > targetOutputVoltage) {
    pwm += (targetOutputVoltage - outputVoltage) * 0.5; // simple p (differential) controller
    outputPowerDelta = 0;
    lastMppt = millis();
    controlMode = CV; // Constant Voltage Mode
    cv = true;
  }

  // Else if current only is above limit: control target = output current! ---
  else if (outputCurrent > maxOutputCurrent) {
    pwm -= (outputCurrent - maxOutputCurrent) * 0.005;
    outputPowerDelta = 0;
    lastMppt = millis();
    controlMode = CC; // Constant Current Mode
    cc = true;
  }

  // Else if current AND voltage are above limit: decrease PWM ---
  else if (outputCurrent > (maxOutputCurrent * 1.2) && outputVoltage > (targetOutputVoltage * 1.2)) {
    pwm --;
    outputPowerDelta = 0;
    lastMppt = millis();
    controlMode = BP; // Battery Protection Mode
    bp = true;
  }

  if (outputCurrent < (maxOutputCurrent * 0.9) && outputVoltage < (targetOutputVoltage * 0.95)) {
    cv = false;
    cc = false;
    bp = false;
  }

  // else: control target = MPPT ---
  if (!cv && !cc && !bp) {
    controlMode = MPPT; // Maximum Power Point Tracking

    // MPPT (max. input power) tracking direction (upwards / downwards is related to panel voltage!)
    if (millis() - lastMppt >= 1000) { // Every 1000ms
      lastMppt = millis();

      // Calculate power and voltage delta
      outputPowerDelta = outputPower - outputPowerPrevious;
      outputVoltageDelta = outputVoltage - outputVoltagePrevious;


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
        if (outputPowerDelta < trackingDirectionChangeWattsThreshold) { // 0.03A current sensor step * 3.5V = 0.1W (0.1 is OK)
          trackingDownwards = !trackingDownwards;
        }
      }

      if (trackingDownwards) targetPanelVoltage -= trackingIncrement;
      else targetPanelVoltage += trackingIncrement;

      // Low power bodge, because MPPT does not track properly, if power is too low
      if (outputPower < lowPowerThreshold) delayHighMaxPanelVoltage = millis();
      if (millis() - delayHighMaxPanelVoltage > 1000) maxPanelVoltage = 17.9;
      else maxPanelVoltage = 14.5;

      if (analogRead(POT) > 2) {
        targetPanelVoltage = analogRead(POT) / 173.38 + 12.0; // 12 - 17.9V // TODO, for MPPT testing! <<<<<<<<<<<<<<<<<<<<
      }

      // Store previous power and voltage for next comparison
      outputPowerPrevious = outputPower;
      outputVoltagePrevious = outputVoltage;
    }

    // Calculate target panel voltage
    static unsigned long lastCalc;
    if (millis() - lastCalc >= 50) { // Every 50ms (prevent it from oscillating in low light conditions)
      lastCalc = millis();
      pwm -= targetPanelVoltage - inputVoltage; // simple p (differential) controller
      //pwm = 255 * targetOutputVoltage / inputVoltage; // only valid for synchronous converter!
    }
  } // End of MPPT ---

  // Write PWM output ----------------------------------------------------------------------------------
  buckConverterDrive();
}

//
// =======================================================================================================
// LED
// =======================================================================================================
//
void led() {
  if (!displayOn) { // LED is only active, if the OLED is off

    if (buckIdle) {
      //LED.on();
      LED.flash(30, 2000, 0, 0);
    }
    else {

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
  }
  else LED.off();
}

//
// =======================================================================================================
// CHECK VCC VOLTAGE
// =======================================================================================================
//
void checkVcc(boolean immediately) {

  static unsigned long lastVcc;
  if (millis() - lastVcc >= 200 || immediately) { // Every 200ms or immediately
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
      if (trackingDownwards) Serial.print("MPPT -");
      else Serial.print("MPPT +");

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
    energy = energy + outputVoltage * outputCurrent / (3600 * (1000 / displayInterval)); // Ws / 3600 = Wh

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

        u8g2.setCursor(79, 32);
        u8g2.print(pwm, 0);

        // Mode & messages
        u8g2.setCursor(79, 64);
        if (buckIdle) {
          u8g2.print("IDLE");
        }
        else {

          if (controlMode == MPPT) {
            if (trackingDownwards) u8g2.print("MPPT -");
            else u8g2.print("MPPT +");
          }
          if (controlMode == CV) u8g2.print("CV");
          if (controlMode == CC) u8g2.print("CC");
          if (controlMode == BP) u8g2.print("BP");
        }

        // SD card presence
        u8g2.setCursor(79, 48);
        if (SDpresent) u8g2.print("SD OK");
        else {
          //else u8g2.print("No SD"); // TODO
          
          //u8g2.print(targetPanelVoltage);
          //u8g2.print("Vt");
          
          u8g2.print("d ");
          u8g2.print(outputPowerDelta);
          
          //u8g2.print(maxPanelVoltage);
          //u8g2.print(flagNegativePower);
        }
      }
    } while ( u8g2.nextPage() ); // show display queue
  }
}

//
// =======================================================================================================
// WRITE SD CARD
// =======================================================================================================
//

void writeSD(boolean noDelay) {
  static unsigned long lastLog;
  if (millis() - lastLog >= loggingInterval || noDelay) {
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
  checkVcc(false); // with delay
  mppt();
  led();
  drawDisplay();
  writeSD(false); // false = with interval timer
}
