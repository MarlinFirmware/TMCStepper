/**
 * stealthChop Example
 *
 * Demonstrates stealthChop mode (extremely quiet PWM-based stepping) on
 * TMC2130/TMC2160/TMC5130/TMC5160 and TMC2208/TMC2209/TMC2224 drivers.
 * Shows how to configure PWM parameters, enable automatic tuning, and compare
 * stealthChop with spreadCycle mode.
 *
 * stealthChop is only available on:
 *   - TMC2130, TMC2160, TMC5130, TMC5160 (SPI drivers)
 *   - TMC2208, TMC2209, TMC2224 (UART drivers)
 *
 * PWM autotuning requires the motor to be moving to generate back EMF.
 *
 * Author: Based on work by Teemu Mäntykallio
 */

#include <TMCStepper.h>

// Pin definitions
#define EN_PIN           38 // Enable
#define DIR_PIN          55 // Direction
#define STEP_PIN         54 // Step
#define CS_PIN           42 // Chip select
#define SW_MOSI          66 // Software Master Out Slave In (MOSI)
#define SW_MISO          44 // Software Master In Slave Out (MISO)
#define SW_SCK           64 // Software Slave Clock (SCK)
#define SW_RX            63 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX            40 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define SERIAL_PORT Serial1 // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS 0b00 // TMC2209 UART slave address (MS1=LOW, MS2=LOW)

// Sensing resistor value - match to your driver
#define R_SENSE 0.11f
  // SilentStepStick series use 0.11
  // UltiMachine Einsy and Archim2 boards use 0.20
  // Panucatt BSD2660 uses 0.10
  // Watterott TMC5160 uses 0.075

// Select your stepper driver type (uncomment one)
//TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

//TMC2160Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC2160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

//TMC5130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC5130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

//TMC5160Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
//TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

//TMC2208Stepper driver(&SERIAL_PORT, R_SENSE);                     // Hardware UART
//TMC2208Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software UART

//TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);     // Hardware UART
//TMC2209Stepper driver(SW_RX, SW_TX, R_SENSE, DRIVER_ADDRESS);     // Software UART

//TMC2224Stepper driver(&SERIAL_PORT, R_SENSE);                     // Hardware UART
//TMC2224Stepper driver(SW_RX, SW_TX, R_SENSE);                     // Software UART

// Default to TMC2130Stepper with hardware SPI
TMC2130Stepper driver(CS_PIN, R_SENSE);

// Safety flag to warn about disconnected belts
bool belt_warning_shown = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to initialize (native USB)
  }

  Serial.println(F("\n========================================"));
  Serial.println(F("  TMCStepper stealthChop Example"));
  Serial.println(F("========================================"));
  Serial.println();

  // Initialize stepper driver pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  // Enable stepper driver (LOW active)
  digitalWrite(EN_PIN, LOW);

  // Initialize communication
  Serial.println(F("Initializing driver..."));

  // For UART drivers, initialize serial communication:
  //SERIAL_PORT.begin(115200);  // Uncomment for TMC2208/2209/2224
  //driver.beginSerial(115200); // Software UART (if selected)

  // Initialize SPI or UART
  driver.begin();

  // Configure basic driver settings
  Serial.println(F("\nConfiguring basic driver settings..."));

  // Set toff (driver off time) - required for stealthChop
  driver.toff(4);

  // Set RMS current (800mA)
  driver.rms_current(800);
  Serial.print(F("  RMS Current: "));
  Serial.print(driver.rms_current());
  Serial.println(" mA");

  // Set microstepping to 1/16
  driver.microsteps(16);
  Serial.print(F("  Microsteps: 1/"));
  Serial.println(driver.microsteps());

  // Test driver connection
  Serial.println(F("\nTesting driver connection..."));
  uint32_t test_result = driver.test_connection();
  if (test_result == 0) {
    Serial.println(F("  Connection test FAILED!"));
    Serial.println(F("  Check wiring and try again."));
    Serial.println();
  } else {
    Serial.println(F("  Connection test PASSED!"));
  }

  // ==========================================
  // stealthChop Configuration
  // ==========================================
  Serial.println(F("\n--- Configuring stealthChop Mode ---"));

  // Enable stealthChop mode (PWM-based quiet stepping)
  // This is only available on TMC2130/2160/5130/5160 and TMC2208/2209/2224
  driver.en_pwm_mode(true);
  Serial.println(F("  stealthChop enabled"));

  // Configure PWM parameters
  // pwm_ampl: PWM amplitude (0-255) - controls motor torque
  driver.pwm_ampl(150);
  Serial.print(F("  PWM Amplitude: "));
  Serial.println(driver.pwm_ampl());

  // pwm_grad: PWM gradient (0-255) - controls current ramp rate
  driver.pwm_grad(10);
  Serial.print(F("  PWM Gradient: "));
  Serial.println(driver.pwm_grad());

  // pwm_freq: PWM frequency selection (0 = 1/2, 1 = 1/4, 2 = 1/8, 3 = 1/16 of SPI clock)
  driver.pwm_freq(1);
  Serial.print(F("  PWM Frequency divisor: 1/"));
  Serial.println(1 << driver.pwm_freq());

  // Enable automatic scaling (required for stealthChop)
  // This automatically adjusts the PWM amplitude based on load
  driver.pwm_autoscale(true);
  Serial.println(F("  PWM autoscale: enabled"));

  // Enable automatic gradient adjustment
  // This automatically adjusts the gradient based on operating conditions
  driver.pwm_autograd(true);
  Serial.println(F("  PWM autograd: enabled"));

  // Wait a moment for settings to take effect
  delay(100);

  // ==========================================
  // Auto-tuning
  // ==========================================
  Serial.println(F("\n--- Running PWM Auto-tuning ---"));
  Serial.println(F("  Motor will run briefly to calibrate PWM parameters..."));
  Serial.println(F("  WARNING: Ensure your belt is securely attached!"));

  // Run motor in one direction to trigger auto-tuning
  // PWM autotuning requires the motor to be moving to generate back EMF
  digitalWrite(DIR_PIN, HIGH);

  // Run 2000 steps to allow auto-tuning to complete
  for (uint16_t i = 2000; i > 0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(100);
  }

  delay(100);

  // Display auto-tuned values
  Serial.println(F("\n  StealthChop Auto-tuning Complete"));

  // Note: pwm_scale_sum, pwm_scale_auto, pwm_ofs_auto, and pwm_grad_auto
  // are only available on TMC2160, TMC2208, TMC2209, TMC2224, TMC2240,
  // TMC5130, and TMC5160 drivers. For TMC2130, these values are not exposed.
  #ifdef HAS_PWM_SCALE
    int16_t pwm_scale_sum = driver.pwm_scale_sum();
    Serial.print(F("    PWM Scale Sum: "));
    Serial.println(pwm_scale_sum);

    #ifdef HAS_PWM_SCALE_AUTO
      int16_t pwm_scale_auto = driver.pwm_scale_auto();
      Serial.print(F("    PWM Scale Auto: "));
      Serial.println(pwm_scale_auto);
    #endif

    #ifdef HAS_PWM_AUTO
      int16_t pwm_ofs_auto = driver.pwm_ofs_auto();
      Serial.print(F("    PWM Offset Auto: "));
      Serial.println(pwm_ofs_auto);

      int16_t pwm_grad_auto = driver.pwm_grad_auto();
      Serial.print(F("    PWM Gradient Auto: "));
      Serial.println(pwm_grad_auto);
    #endif
  #else
    Serial.println(F("    (Auto-tune values not available on this driver type)"));
  #endif

  // ==========================================
  // Mode Comparison
  // ==========================================
  Serial.println(F("\n--- Comparing stealthChop vs spreadCycle ---"));

  // Show current stealthChop status
  bool stealth_enabled = driver.en_pwm_mode();
  Serial.print(F("  stealthChop enabled: "));
  Serial.println(stealth_enabled ? "Yes" : "No");

  // Switch to spreadCycle mode for comparison
  Serial.println(F("\n  Switching to spreadCycle mode..."));
  driver.en_pwm_mode(false);
  driver.en_spreadCycle(true);
  Serial.println(F("  spreadCycle enabled"));

  // Run motor briefly in spreadCycle mode
  Serial.println(F("  Running motor in spreadCycle mode..."));
  digitalWrite(DIR_PIN, LOW);
  for (uint16_t i = 1000; i > 0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(100);
  }

  // Switch back to stealthChop mode
  Serial.println(F("\n  Switching back to stealthChop mode..."));
  driver.en_pwm_mode(true);
  driver.pwm_autoscale(true);
  driver.pwm_autograd(true);
  Serial.println(F("  stealthChop re-enabled with auto-tuning"));

  // ==========================================
  // Status Display
  // ==========================================
  Serial.println(F("\n--- Current Driver Status ---"));

  // Read DRV_STATUS register
  uint32_t drv_status = driver.DRV_STATUS();
  Serial.print(F("  DRV_STATUS: 0x"));
  Serial.println(drv_status, HEX);

  // Get current value
  Serial.print(F("  Current: "));
  Serial.print(driver.rms_current());
  Serial.println(" mA");

  // Check if in stealthChop mode
  Serial.print(F("  stealthChop active: "));
  Serial.println(driver.en_pwm_mode() ? "Yes" : "No");

  // ==========================================
  // Safety Warning
  // ==========================================
  static bool belt_warning_shown = false;
  if (!belt_warning_shown) {
    Serial.println(F("\n========================================"));
    Serial.println(F("  SAFETY NOTE:"));
    Serial.println(F("  stealthChop mode is much quieter than"));
    Serial.println(F("  spreadCycle, but if your belt comes"));
    Serial.println(F("  off, the motor will stop making noise"));
    Serial.println(F("  while the X-axis continues moving."));
    Serial.println(F("  Always ensure your belts are secure!"));
    Serial.println(F("========================================"));
    belt_warning_shown = true;
  }

  Serial.println(F("\nEntering main loop - motor will alternate directions."));
  Serial.println();
}

void loop() {
  static bool shaft_direction = false;

  // Run 5000 steps in current direction
  for (uint16_t i = 5000; i > 0; i--) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(160);  // ~3125 steps/second
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(160);
  }

  // Switch direction (shaft() is only available on TMC5130/5160)
  // Uncomment the following lines if using a TMC5130 or TMC5160 driver:
  // shaft_direction = !shaft_direction;
  // driver.shaft(shaft_direction);

  // Print status every 10 direction changes
  static uint8_t count = 0;
  if (++count >= 10) {
    count = 0;
    Serial.println(F("\n--- Status Update ---"));
    Serial.print(F("  DRV_STATUS: 0x"));
    Serial.println(driver.DRV_STATUS(), HEX);
    Serial.print(F("  Current: "));
    Serial.print(driver.rms_current());
    Serial.println(" mA");
    Serial.print(F("  stealthChop: "));
    Serial.println(driver.en_pwm_mode() ? "Yes" : "No");
    #ifdef HAS_PWM_SCALE
      Serial.print(F("  PWM Scale Sum: "));
      Serial.println(driver.pwm_scale_sum());
    #endif
    Serial.println();
  }

  delay(100);
}
