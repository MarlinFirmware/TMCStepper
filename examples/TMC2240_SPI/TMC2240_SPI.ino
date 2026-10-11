/**
 * TMC2240 SPI Example
 *
 * Demonstrates the TMC2240 stepper driver using SPI communication.
 * The TMC2240 is a versatile driver with unique features including:
 * - StallGuard4 (SG4) for load sensing
 * - ADC-based temperature and voltage monitoring
 * - Over-temperature and overvoltage pre-warnings
 * - StealthChop for quiet operation
 * - High current capability
 *
 * Connect your TMC2240 driver via SPI:
 *   CS_PIN  -> Chip Select
 *   MOSI    -> Master Out Slave In
 *   MISO    -> Master In Slave Out
 *   SCK     -> SPI Clock
 *
 * Safety Warning:
 *   Always ensure your belts are properly secured before testing!
 *   Disconnect the motor when not in use to prevent unexpected movement.
 *   Verify your power supply voltage is within the driver's specifications.
 */

#include <TMCStepper.h>

// Pin definitions
#define EN_PIN           38 // Enable pin (LOW = enabled)
#define DIR_PIN          55 // Direction pin
#define STEP_PIN         54 // Step pin
#define CS_PIN           42 // Chip select pin

// Software SPI pins (alternative to hardware SPI)
#define SW_MOSI          66 // Software Master Out Slave In (MOSI)
#define SW_MISO          44 // Software Master In Slave Out (MISO)
#define SW_SCK           64 // Software Slave Clock (SCK)

// Sense resistor value - match to your driver board
#define R_SENSE 0.11f // Typical value for SilentStepStick series

// Serial baud rate for monitoring
#define SERIAL_BAUD 250000

// Motor parameters
#define MAX_SPEED      10000  // Maximum step frequency (Hz)
#define STEPS_PER_REV  200    // Steps per motor revolution

// Select your TMC2240 configuration
// Option 1: Hardware SPI (default, uncommented)
TMC2240Stepper driver(CS_PIN, R_SENSE);

// Option 2: Software SPI (uncomment if needed)
// TMC2240Stepper driver(CS_PIN, SW_MOSI, SW_MISO, SW_SCK);

// StallGuard4 threshold (range: -64 to 63)
#define SG4_THRESHOLD 50

// Current settings
#define RUN_CURRENT_mA  1200  // Run current in mA
#define HOLD_MULTIPLIER 0.5   // Hold current as fraction of run current

// Over-temperature pre-warn threshold (Celsius)
#define OTPW_THRESHOLD 105.0

// Overvoltage threshold (Volts)
#define OV_THRESHOLD 30.0

// Timing for monitoring updates (milliseconds)
#define MONITOR_INTERVAL 500

// Forward declaration
void print_stealthchop_status();
void print_sg4_status();
void print_adc_readings();
void print_drv_status();

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    delay(10);  // Wait for serial port to connect (needed for native USB)
  }

  Serial.println(F("\n========================================"));
  Serial.println(F("  TMC2240 SPI Example"));
  Serial.println(F("========================================"));
  Serial.println(F("\nInitializing..."));

  // Configure GPIO pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  // Enable driver in hardware (LOW = enabled)
  digitalWrite(EN_PIN, LOW);

  // Initialize SPI
  SPI.begin();

  // Initialize the TMC2240 driver
  driver.begin();

  // Test connection to verify communication
  Serial.print(F("Connection test: "));
  uint8_t connection_status = driver.test_connection();
  if (connection_status == 0) {
    Serial.println(F("OK"));
  } else {
    Serial.print(F("FAILED (error code: "));
    Serial.print(connection_status);
    Serial.println(F(")"));
    Serial.println(F("WARNING: Check SPI connections!"));
  }

  // Reset to factory defaults for clean start
  driver.defaults();
  delay(10);

  // ============================================
  // Basic Motor Configuration
  // ============================================

  // Enable driver in software (TOFF controls internal enable)
  driver.toff(4);  // Enable stepper driver (value 1-15)

  // Set RMS current
  driver.rms_current(RUN_CURRENT_mA, HOLD_MULTIPLIER);

  // Set microstepping (must be 256, 128, 64, 32, 16, 8, 4, 2, or 0)
  driver.microsteps(STEPS_PER_REV / 2);  // 1/2 microstepping = 100 microsteps

  // Configure IHOLD/IRUN separately (alternative to rms_current)
  driver.iholddelay(1);    // Delay before switching to hold current

  Serial.println(F("\n--- Basic Configuration ---"));
  Serial.print(F("  Run Current: "));
  Serial.print(RUN_CURRENT_mA);
  Serial.println(F(" mA"));
  Serial.print(F("  Hold Current: "));
  Serial.print(RUN_CURRENT_mA * HOLD_MULTIPLIER);
  Serial.println(F(" mA"));
  Serial.print(F("  Microsteps: "));
  Serial.println(driver.microsteps());

  // ============================================
  // CHOPCONF Configuration
  // ============================================

  // Chopper configuration
  driver.TBL(2);           // Blank time (2 = 16ns, typical)
  driver.chm(false);       // Chopper mode: false = spreadCycle, true = fullstep
  driver.intpol(true);     // Interpolate to 256 microsteps
  driver.dedge(true);      // Double edge stepping
  driver.hysteresis_end(-3);  // Hysteresis end value (can be negative)
  driver.hysteresis_start(3); // Hysteresis start value

  Serial.println(F("\n--- CHOPCONF Configuration ---"));
  Serial.print(F("  Blank Time: "));
  Serial.println(driver.TBL());
  Serial.print(F("  Interpolation: "));
  Serial.println(driver.intpol() ? F("Enabled") : F("Disabled"));
  Serial.print(F("  Double Edge: "));
  Serial.println(driver.dedge() ? F("Enabled") : F("Disabled"));

  // ============================================
  // StealthChop (PWM) Mode
  // ============================================

  // Enable StealthChop for quiet operation
  driver.en_pwm_mode(true);

  // PWM configuration
  driver.pwm_autoscale(true);   // Auto-calculate PWM offset
  driver.pwm_autograd(true);    // Auto-calculate PWM gradient
  driver.pwm_ofs(20);           // PWM offset (0-255)
  driver.pwm_grad(10);          // PWM gradient (0-255)
  driver.pwm_freq(1);           // PWM frequency selection

  Serial.println(F("\n--- StealthChop Configuration ---"));
  Serial.print(F("  Mode: "));
  Serial.println(driver.en_pwm_mode() ? F("Enabled") : F("Disabled"));

  // ============================================
  // StallGuard4 (SG4) Configuration
  // ============================================

  // Enable StallGuard4
  driver.sg4_en(true);          // Enable SG4 mode
  driver.sg4_filt_en(true);     // Enable input filtering for better accuracy

  // Set StallGuard4 threshold (range: -64 to 63, higher = less sensitive)
  driver.sgt(SG4_THRESHOLD);

  // Set CoolStep threshold (speed above which SG4 is active)
  driver.TCOOLTHRS(0xFFFFF);    // Maximum threshold for high-speed operation

  // SG4 additional settings
  driver.sg4_smin(0);           // Minimum speed for SG4 evaluation
  driver.sg4_smax(100);         // Maximum speed for SG4 evaluation
  driver.sg4_thrs(50);          // SG4 threshold for additional features

  Serial.println(F("\n--- StallGuard4 Configuration ---"));
  print_sg4_status();

  // ============================================
  // Over-Temperature and Overvoltage Protection
  // ============================================

  // Set over-temperature pre-warn threshold
  driver.set_overtemp_prewarn_celsius(OTPW_THRESHOLD);

  // Set overvoltage threshold
  driver.set_overvoltage_threshold_voltage(OV_THRESHOLD);

  Serial.println(F("\n--- Protection Settings ---"));
  Serial.print(F("  OTPreWarn Threshold: "));
  Serial.print(OTPW_THRESHOLD);
  Serial.println(F(" °C"));
  Serial.print(F("  Overvoltage Threshold: "));
  Serial.print(OV_THRESHOLD);
  Serial.println(F(" V"));

  // Save configuration to driver
  driver.push();

  Serial.println(F("\nConfiguration complete!"));
  Serial.println(F("Monitoring readings (type 's' to toggle StealthChop, 'r' to reset):\n"));

  // Start motor movement
  digitalWrite(DIR_PIN, HIGH);  // Set direction
}

void loop() {
  static uint32_t last_monitor = 0;
  static bool stealthchop_enabled = true;
  static bool direction = HIGH;
  static uint32_t step_count = 0;

  // Handle serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    switch (cmd) {
      case 's':
      case 'S':
        // Toggle StealthChop mode
        stealthchop_enabled = !stealthchop_enabled;
        driver.en_pwm_mode(stealthchop_enabled);
        driver.push();
        Serial.print(F("\nStealthChop: "));
        Serial.println(stealthchop_enabled ? F("Enabled") : F("Disabled"));
        break;

      case 'r':
      case 'R':
        // Reset to defaults and reconfigure
        Serial.println(F("\nResetting to defaults..."));
        driver.defaults();
        delay(10);

        // Re-apply configuration
        driver.toff(4);
        driver.rms_current(RUN_CURRENT_mA, HOLD_MULTIPLIER);
        driver.microsteps(STEPS_PER_REV / 2);
        driver.en_pwm_mode(true);
        driver.pwm_autoscale(true);
        driver.pwm_autograd(true);
        driver.sg4_en(true);
        driver.sg4_filt_en(true);
        driver.sgt(SG4_THRESHOLD);
        driver.set_overtemp_prewarn_celsius(OTPW_THRESHOLD);
        driver.set_overvoltage_threshold_voltage(OV_THRESHOLD);
        driver.push();

        Serial.println(F("Reconfiguration complete.\n"));
        break;

      case '+':
        // Increase speed (decrease period)
        Serial.println(F("Speed increase - modify period variable directly"));
        break;

      case '-':
        // Decrease speed (increase period)
        Serial.println(F("Speed decrease - modify period variable directly"));
        break;

      case 'd':
      case 'D':
        // Toggle direction
        direction = !direction;
        driver.shaft(direction);
        Serial.print(F("Direction: "));
        Serial.println(direction ? F("Forward") : F("Reverse"));
        break;
    }
  }

  // Run motor - simple step generation
  {
    static uint32_t last_step = 0;
    static uint32_t period = 500;  // Step period in microseconds

    uint32_t now = micros();
    if (now - last_step >= period) {
      last_step = now;

      // Generate step pulse
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(1);
      digitalWrite(STEP_PIN, LOW);

      step_count++;
    }
  }

  // Periodic monitoring
  uint32_t now = millis();
  if (now - last_monitor >= MONITOR_INTERVAL) {
    last_monitor = now;

    print_adc_readings();
    print_drv_status();

    if (stealthchop_enabled) {
      print_stealthchop_status();
    }

    print_sg4_status();

    Serial.println();
  }
}

/**
 * Print ADC-based readings (temperature, voltage)
 */
void print_adc_readings() {
  // Get chip temperature (uses internal ADC)
  float temp = driver.get_chip_temperature();
  Serial.print(F("Temp: "));
  Serial.print(temp, 1);
  Serial.print(F("°C | VSUP: "));

  // Get supply voltage (uses internal ADC)
  float vsup = driver.get_vsupply_voltage();
  Serial.print(vsup, 1);
  Serial.print(F("V | AIN: "));

  // Get AIN pin voltage (external ADC input)
  float ain = driver.get_ain_voltage();
  Serial.print(ain, 3);
  Serial.print(F("V | OTPW: "));

  // Check protection thresholds
  float otpw = driver.get_overtemp_prewarn_celsius();
  Serial.print(otpw, 1);
  Serial.print(F("°C | OVTh: "));

  float ovth = driver.get_overvoltage_threshold_voltage();
  Serial.print(ovth, 1);
  Serial.println(F("V"));
}

/**
 * Print StealthChop/PWM status
 */
void print_stealthchop_status() {
  Serial.print(F("PWM: "));
  Serial.print(driver.en_pwm_mode() ? F("Enabled") : F("Disabled"));

  // PWM scale values (read from PWM_SCALE register)
  uint8_t scale_sum = driver.pwm_scale_sum();
  int16_t scale_auto = driver.pwm_scale_auto();
  uint8_t ofs_auto = driver.pwm_ofs_auto();
  uint8_t grad_auto = driver.pwm_grad_auto();

  Serial.print(F(" | SSum: "));
  Serial.print(scale_sum);
  Serial.print(F(" | SAuto: "));
  Serial.print(scale_auto);
  Serial.print(F(" | OAuto: "));
  Serial.print(ofs_auto);
  Serial.print(F(" | GAuto: "));
  Serial.println(grad_auto);
}

/**
 * Print StallGuard4 status and result
 */
void print_sg4_status() {
  Serial.print(F("SG4: "));
  Serial.print(driver.sg4_en() ? F("On") : F("Off"));

  // Read StallGuard4 result (SG4_RESULT register)
  uint16_t sg4_result = driver.sg4_result();
  Serial.print(F(" | SG4 Result: "));
  Serial.print(sg4_result);

  // Read StallGuard threshold
  int8_t sgt = driver.sgt();
  Serial.print(F(" | SG4 Thrs: "));
  Serial.println(sgt);
}

/**
 * Print DRV_STATUS register flags
 */
void print_drv_status() {
  // Check individual flags
  Serial.print(F("Stall: "));
  Serial.print(driver.stst() ? F("Yes") : F("No"));
  Serial.print(F(" | OT: "));
  Serial.print(driver.ot() ? F("Yes") : F("No"));
  Serial.print(F(" | OTPW: "));
  Serial.print(driver.otpw() ? F("Yes") : F("No"));
  Serial.print(F(" | OL-A: "));
  Serial.print(driver.ola() ? F("Yes") : F("No"));
  Serial.print(F(" | OL-B: "));
  Serial.print(driver.olb() ? F("Yes") : F("No"));
  Serial.print(F(" | S2G-A: "));
  Serial.print(driver.s2ga() ? F("Yes") : F("No"));
  Serial.print(F(" | S2G-B: "));
  Serial.println(driver.s2gb() ? F("Yes") : F("No"));

  // Current scale actual
  uint16_t cs_actual = driver.CS_ACTUAL();
  Serial.print(F("CS: "));
  Serial.print(cs_actual);
  Serial.print(F(" | MSCNT: "));
  Serial.println(driver.MSCNT());
}