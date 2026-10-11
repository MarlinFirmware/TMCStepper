/**
 * Calibrate spreadCycle Parameters
 *
 * This example helps calibrate the spreadCycle chopper configuration
 * for TMC stepper drivers (TMC2130, TMC2160, TMC5130, TMC5160, etc.).
 *
 * The procedure tunes the off-time and hysteresis settings by
 * running the motor and letting you adjust parameters while listening
 * to the motor's audible noise. The goal is to find a good balance
 * between noise, heating, and stall performance.
 *
 * Hardware notes:
 *   - This example defaults to RAMPS 1.4 pinout. Adjust pins for your board.
 *   - The belt should be disconnected from the motor during calibration
 *     since this sketch does not respect endstops or position limits.
 *
 * Written by Teemu Mantykallio.
 */

#include <TMCStepper.h>

// ─── Pin Definitions ───────────────────────────────────────────────────
// Default for RAMPS 1.4 (X-axis). Change for your board.

#define EN_PIN   38  // Enable (LOW active)
#define DIR_PIN  55  // Direction
#define STEP_PIN 54  // Step
#define CS_PIN   42  // Chip select

// Software SPI pins (uncomment if using software SPI)
// #define SW_MOSI  66
// #define SW_MISO  44
// #define SW_SCK   64

// ─── Sense Resistor ────────────────────────────────────────────────────
// Match the R_SENSE value for your driver board.
//   SilentStepStick series        -> 0.11
//   UltiMachine Einsy / Archim 2  -> 0.20
//   Panucatt BSD2660              -> 0.10
//   Watterott TMC5160             -> 0.075
#define R_SENSE 0.11f

// ─── Select Your Driver ────────────────────────────────────────────────
// Uncomment exactly ONE line below for your driver type.
// Hardware SPI is preferred; software SPI is available as an option.

// TMC2130Stepper driver = TMC2130Stepper(CS_PIN, R_SENSE);
// TMC2130Stepper driver = TMC2130Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

// TMC2660Stepper driver = TMC2660Stepper(CS_PIN, R_SENSE);
// TMC2660Stepper driver = TMC2660Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

// TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE);
// TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

// ─── Initial Configuration ────────────────────────────────────────────
// You may override these defaults before running.
// Blank time options: 16, 24, 36, or 54 cycles.
// Off time range: 1-15.
// Hysteresis start range: 1-8.
// Hysteresis end range: -3 to 12.

struct {
  uint8_t  blank_time       = 24;
  uint8_t  off_time         = 3;
  uint8_t  hysteresis_start = 1;
  int8_t   hysteresis_end   = 12;
} config;

// ─── Helper Functions ──────────────────────────────────────────────────

void reportCurrentSettings() {
  Serial.print(F("Off time = "));
  Serial.print(config.off_time);
  Serial.print(F("  Hysteresis end = "));
  Serial.print(config.hysteresis_end);
  Serial.print(F("  Hysteresis start = "));
  Serial.println(config.hysteresis_start);
}

void initPins() {
  pinMode(EN_PIN,   OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN,   OUTPUT);

  digitalWrite(EN_PIN,   HIGH);  // deactivate driver (LOW active)
  digitalWrite(DIR_PIN,  LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(CS_PIN,   HIGH);

  SPI.begin();
  pinMode(MISO, INPUT_PULLUP);
}

// ─── Setup ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(250000);
  while (!Serial) { delay(10); }  // wait for serial port to initialize

  Serial.println(F("\n============================================="));
  Serial.println(F("  Calibrate spreadCycle Parameters"));
  Serial.println(F("============================================="));
  Serial.println(F("This example uses the X-axis driver on a RAMPS 1.4."));
  Serial.println(F("Make sure your serial terminal sends NEWLINE only!"));
  Serial.println();

  // Brief pause so the user can open the serial monitor
  for (uint8_t i = 0; i < 60; i++) { Serial.print('.'); delay(100); }
  Serial.println();

  // ── Safety: confirm belt is disconnected ──
  Serial.println(F("First, disconnect the belt from the motor."));
  Serial.println(F("We will not respect endstops or position limits."));
  while (true) {
    Serial.println(F("Is the belt disconnected? Send 'yes' to confirm."));
    while (!Serial.available());
    String yn = Serial.readStringUntil('\n');
    if (yn == "yes") break;
    Serial.println(F("Please disconnect the belt and press 'yes' again."));
  }

  // ── Safety: confirm power is on ──
  Serial.println(F("\nNow make sure the driver has 12 V (or greater) power."));
  while (true) {
    Serial.println(F("Is VMOT power on? Send 'yes' to confirm."));
    while (!Serial.available());
    String yn = Serial.readStringUntil('\n');
    if (yn == "yes") break;
    Serial.println(F("Please turn on power and press 'yes' again."));
  }

  // ── Test connection ──
  Serial.print(F("\nTesting connection... "));
  initPins();
  uint8_t result = driver.test_connection();
  if (result) {
    Serial.println(F("FAILED!"));
    Serial.print(F("  Likely cause: "));
    switch (result) {
      case 1: Serial.println(F("loose connection")); break;
      case 2: Serial.println(F("no power to driver"));  break;
      default: Serial.println(F("unknown error"));       break;
    }
    Serial.println(F("Fix the problem and reset the board."));
    while (true);  // halt
  }
  Serial.println(F("OK"));

  // ── Configure driver ──
  driver.push();
  driver.microsteps(256);
  driver.irun(10);
  driver.ihold(10);

  Serial.print(F("Blank time = "));
  Serial.print(config.blank_time);
  Serial.println(F(" cycles"));
  driver.blank_time(config.blank_time);

  Serial.print(F("Off time = "));
  Serial.println(config.off_time);
  driver.toff(config.off_time);

  Serial.print(F("Hysteresis end = "));
  Serial.println(config.hysteresis_end);
  driver.hysteresis_end(config.hysteresis_end);

  Serial.print(F("Hysteresis start = "));
  Serial.println(config.hysteresis_start);
  driver.hysteresis_start(config.hysteresis_start);

  Serial.print(F("Effective hysteresis = "));
  Serial.print(config.hysteresis_end);
  Serial.print(F(" + "));
  Serial.print(config.hysteresis_start);
  Serial.print(F(" = "));
  Serial.println(config.hysteresis_end + config.hysteresis_start);

  // ── Start the motor ──
  Serial.println(F("\nStarting the motor..."));
  digitalWrite(EN_PIN, LOW);  // enable driver
  while (driver.cur_a() < 240) {
    digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN, LOW);
    delay(3);
  }

  // ── Interactive calibration ──
  Serial.println(F("\nListening to the motor:"));
  Serial.println(F("  Press  +  to increase hysteresis end"));
  Serial.println(F("  Press  -  to decrease hysteresis end"));
  Serial.println(F("  Press  =  to finish calibration"));
  Serial.println();

  while (true) {
    if (Serial.available() > 0) {
      uint8_t c = Serial.read();
      if (c == '+') {
        if (config.hysteresis_end >= 12)
          Serial.println(F("Hysteresis end is already at MAX (12)."));
        else {
          config.hysteresis_end++;
          reportCurrentSettings();
          driver.hysteresis_end(config.hysteresis_end);
        }
      }
      else if (c == '-') {
        if (config.hysteresis_end <= -3)
          Serial.println(F("Hysteresis end is already at MIN (-3)."));
        else {
          config.hysteresis_end--;
          reportCurrentSettings();
          driver.hysteresis_end(config.hysteresis_end);
        }
      }
      else if (c == '=') {
        break;
      }
    }
  }

  // ── Final results ──
  Serial.println();
  Serial.print(F("Final effective hysteresis = "));
  Serial.print(config.hysteresis_end);
  Serial.print(F(" + "));
  Serial.print(config.hysteresis_start);
  Serial.print(F(" = "));
  Serial.println(config.hysteresis_end + config.hysteresis_start);

  Serial.println(F("\nYour Marlin configuration parameters:"));
  Serial.print(F("#define CHOPPER_TIMING { "));
  Serial.print(config.off_time);
  Serial.print(F(", "));
  Serial.print(config.hysteresis_end);
  Serial.print(F(", "));
  Serial.print(config.hysteresis_start);
  Serial.println(F(" }"));

  Serial.println(F("\nCalibration complete. Good luck!"));
}

// ─── Loop ──────────────────────────────────────────────────────────────

void loop() {
  // Nothing to do here.
}
