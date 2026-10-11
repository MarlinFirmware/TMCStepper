/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2209 motor load using the StallGuard value.
 * You can fine-tune the reading by changing the STALL_VALUE.
 * This lets you control at which load the value will read 0
 * and the stall flag will be triggered (pin DIAG1 goes high).
 * A higher STALL_VALUE makes the reading less sensitive.
 * A lower STALL_VALUE makes it more sensitive.
 *
 * You can control the rotation speed via serial:
 *   0  Stop
 *   1  Resume
 *   +  Speed up
 *   -  Slow down
 */
#include <TMCStepper.h>

#define MAX_TIMER       40  // Minimum timer value (fastest speed)
#define MIN_TIMER    10000  // Maximum timer value (slowest speed)

#define STALL_VALUE     100 // [0..255]

#define EN_PIN           38 // Enable
#define DIR_PIN          55 // Direction
#define STEP_PIN         54 // Step
#define SW_RX            63 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS  0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

ISR(TIMER1_COMPA_vect) {
  //STEP_PORT ^= 1 << STEP_BIT_POS;
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
}

void setup() {
  Serial.begin(250000);         // Init serial port and set baudrate
  while (!Serial);              // Wait for serial port to connect
  Serial.println("\nStart...");

  SERIAL_PORT.begin(115200);
  driver.beginSerial(115200);

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400);      // mA
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF);    // 20-bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);

  // Set stepper interrupt
  {
    cli();                      // Stop interrupts
    TCCR1A = 0;                 // Set entire TCCR1A register to 0
    TCCR1B = 0;                 // Set entire TCCR1B register to 0
    TCNT1  = 0;                 // Initialize counter value to 0
    OCR1A = 256;                // = (16*10^6) / (1*1024) - 1 (must be <65536)
    TCCR1B |= (1 << WGM12);     // Turn on CTC mode
    TCCR1B |= (1 << CS11);      // Set CS11 bits for 8 prescaler
    TIMSK1 |= (1 << OCIE1A);    // Enable timer compare interrupt
    sei();                      // Allow interrupts
  }
}

void loop() {
  static uint32_t last_time = 0;
  uint32_t ms = millis();

  while (Serial.available() > 0) {
    int8_t read_byte = Serial.read();
    if (read_byte == '0') {
      TIMSK1 &= ~(1 << OCIE1A);
      digitalWrite(EN_PIN, HIGH);
    } else if (read_byte == '1') {
      TIMSK1 |=  (1 << OCIE1A);
      digitalWrite(EN_PIN, LOW);
    } else if (read_byte == '+') {
      if (OCR1A > MAX_TIMER) OCR1A -= 20;
    } else if (read_byte == '-') {
      if (OCR1A < MIN_TIMER) OCR1A += 20;
    }
  }

  if ((ms - last_time) > 100) {  // Run every 0.1s
    last_time = ms;

    Serial.print("0 ");
    Serial.print(driver.SG_RESULT(), DEC);
    Serial.print(" ");
    Serial.println(driver.cs2rms(driver.cs_actual()), DEC);
  }
}