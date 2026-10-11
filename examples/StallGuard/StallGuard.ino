/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2130 or TMC5160 motor load using the stallGuard value.
 * You can fine-tune the reading by changing the STALL_VALUE.
 * This lets you control at which load the value will read 0
 * and the stall flag will be triggered. This also sets pin DIAG1 high.
 * A higher STALL_VALUE makes the reading less sensitive,
 * a lower STALL_VALUE makes it more sensitive.
 *
 * Control the rotation speed via serial:
 *   0   Stop
 *   1   Resume
 *   +   Speed up
 *   -   Slow down
 */
#include <TMCStepper.h>

// Timer interval values (inverted relationship to speed: smaller = faster)
#define TIMER_INTERVAL_MAX  40 // Fastest stepping (smallest interval)
#define TIMER_INTERVAL_MIN 1000 // Slowest stepping (largest interval)

#define STALL_VALUE 15 // [-64..63]

#define EN_PIN     38 // Enable
#define DIR_PIN    55 // Direction
#define STEP_PIN   54 // Step
#define CS_PIN     42 // Chip select
#define SW_MOSI    66 // Software Master Out Slave In (MOSI)
#define SW_MISO    44 // Software Master In Slave Out (MISO)
#define SW_SCK     64 // Software Slave Clock (SCK)

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
//TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
//TMC5160Stepper driver(CS_PIN, R_SENSE);
//TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

using namespace TMC2130_n;

void setup() {
  SPI.begin();
  Serial.begin(250000);          // Init serial port and set baud rate
  while (!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400);       // mA
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF);    // 20-bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  // Set stepper interrupt using Timer1 CTC mode
  {
    cli();                       // Stop interrupts
    TCCR1A = 0;                  // Clear TCCR1A
    TCCR1B = 0;                  // Clear TCCR1B
    TCNT1 = 0;                   // Initialize counter value
    OCR1A = 256;                 // = (16*10^6) / (1*1024) - 1 (must be <65536)
    TCCR1B |= (1 << WGM12);     // Turn on CTC mode
    TCCR1B |= (1 << CS11);      // Set CS11 bit for 8 prescaler
    TIMSK1 |= (1 << OCIE1A);    // Enable timer compare interrupt
    sei();                       // Allow interrupts
  }
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t ms = millis();

  while (Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    if (read_byte == '0') {
      TIMSK1 &= ~(1 << OCIE1A);
      digitalWrite(EN_PIN, HIGH);
    }
    else if (read_byte == '1') {
      TIMSK1 |= (1 << OCIE1A);
      digitalWrite(EN_PIN, LOW);
    }
    else if (read_byte == '+') {
      if (OCR1A > TIMER_INTERVAL_MAX) OCR1A -= 20;
    }
    else if (read_byte == '-') {
      if (OCR1A < TIMER_INTERVAL_MIN) OCR1A += 20;
    }
  }

  if (ms - last_time > 100) {    // Run every 0.1s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();

    Serial.print("0 ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
  }
}