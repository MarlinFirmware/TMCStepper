/**
 * Author Teemu Mäntykallio
 *
 * This example demonstrates how to use StallGuard with a TMC2660 stepper
 * driver. It reads the motor load (stallGuard value) and prints it over
 * serial at 250000 baud.
 *
 * Control the rotation speed via serial:
 *   0  Stop
 *   1  Resume
 *   +  Speed up
 *   -  Slow down
 *
 * You can fine-tune the StallGuard sensitivity by changing STALL_VALUE.
 * A higher STALL_VALUE makes the reading less sensitive, while a lower
 * value makes it more sensitive. The stall flag will trigger when the
 * motor load exceeds this threshold, setting DIAG1 high.
 */

#include <TMCStepper.h>

#define MAX_SPEED       40    // Minimum OCR1A value (fastest speed)
#define MIN_SPEED     1000    // Maximum OCR1A value (slowest speed)
#define STALL_VALUE      15   // [-64..63] StallGuard threshold

#define EN_PIN          38    // Enable pin
#define DIR_PIN         55    // Direction pin
#define STEP_PIN        54    // Step pin
#define CS_PIN          42    // Chip select pin

#define R_SENSE       0.11f   // Match to your driver
                              // SilentStepStick series use 0.11
                              // UltiMachine Einsy and Archim2 boards use 0.2
                              // Panucatt BSD2660 uses 0.1
                              // Watterott TMC5160 uses 0.075

// Instantiate the TMC2660 driver (Hardware SPI)
TMC2660Stepper driver(CS_PIN, R_SENSE);

using namespace TMC2660_n;

// Using direct register manipulation can reach faster stepping times.
// For STEP_PIN = 54 on an ATmega2560, the port/bit mapping is:
#define STEP_PORT       PORTF
#define STEP_BIT_POS        0

ISR(TIMER1_COMPA_vect) {
  // Direct register toggle for maximum speed:
  // STEP_PORT ^= 1 << STEP_BIT_POS;
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}

void setup() {
  SPI.begin();
  Serial.begin(250000);
  while (!Serial);

  Serial.println("\n[TMC2660 StallGuard Example]");

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  // Initialize the TMC2660 driver
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400);   // 400 mA
  driver.microsteps(16);
  driver.sfilt(true);        // Filter improves StallGuard readout
  driver.rdsel(0b01);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  // Configure Timer1 CTC mode for stepper interrupt
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = 256;              // = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);    // CTC mode
  TCCR1B |= (1 << CS11);     // 8x prescaler
  TIMSK1 |= (1 << OCIE1A);   // Enable compare interrupt
  sei();
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t ms = millis();

  // Handle serial commands
  while (Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    if (read_byte == '0') {
      TIMSK1 &= ~(1 << OCIE1A);  // Disable stepper interrupt
      driver.toff(0);            // Disable driver output
    }
    else if (read_byte == '1') {
      TIMSK1 |=  (1 << OCIE1A);  // Enable stepper interrupt
      driver.toff(driver.savedToff());  // Restore driver output
    }
    else if (read_byte == '+') {
      if (OCR1A > MAX_SPEED) OCR1A -= 20;
    }
    else if (read_byte == '-') {
      if (OCR1A < MIN_SPEED) OCR1A += 20;
    }
  }

  // Print StallGuard reading every 100ms
  if (ms - last_time > 100) {
    last_time = ms;

    READ_RDSEL10_t data{0};
    data.sr = driver.DRV_STATUS();

    Serial.print("0 ");
    Serial.print(data.sg_result, DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(data.se), DEC);
  }
}
