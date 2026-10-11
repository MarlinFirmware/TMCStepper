/**
 * TMC2240 UART Example
 *
 * Demonstrates the TMC2240 stepper driver using UART communication.
 * The TMC2240 is a versatile driver with unique features including:
 * - UART-based control (no STEP/DIR pins needed!)
 * - VACTUAL speed control via UART
 * - StallGuard4 (SG4) for load sensing
 * - ADC-based temperature and voltage monitoring
 * - Over-temperature and overvoltage pre-warnings
 * - StealthChop for quiet operation
 * - High current capability
 * - Daisy-chain node configuration
 *
 * UART connections:
 *   SW_TX  -> TMC2240 UART TX (DIN)
 *   SW_RX  -> TMC2240 UART RX (DOUT)
 *
 * For Hardware UART, connect to any Serial port on your board.
 *
 * Safety Warning:
 *   Always ensure your belts are properly secured before testing!
 *   Disconnect the motor when not in use to prevent unexpected movement.
 *   Verify your power supply voltage is within the driver's specifications.
 */

#include <TMCStepper.h>

// Pin definitions
#define EN_PIN           38 // Enable pin (LOW = enabled)
#define STEP_PIN         54 // Step pin (optional, for STEP input mode)
#define DIR_PIN          55 // Direction pin (optional, for STEP input mode)
#define CS_PIN           42 // Chip select (for SPI mode comparison)
#define INDEX_PIN        2  // INDEX pin on TMC driver (interrupt pin)

// Software UART pins (alternative to hardware UART)
#define SW_RX            63 // TMC2240 SoftwareSerial receive pin
#define SW_TX            40 // TMC2240 SoftwareSerial transmit pin

// Hardware UART port
#define SERIAL_PORT Serial1 // TMC2240 HardwareSerial port

// Sense resistor value - match to your driver board
#define R_SENSE 0.11f // Typical value for SilentStepStick series

// Serial baud rate for PC monitoring
#define PC_SERIAL_BAUD 250000

// UART baud rate for TMC2240 communication
#define TMC_UART_BAUD 115200

// Motor parameters
#define MAX_SPEED      20000  // Maximum VACTUAL speed
#define MIN_SPEED      10000  // Minimum VACTUAL speed
#define STEPS_PER_REV  200    // Steps per motor revolution
#define DEFAULT_SPEED  5000   // Default VACTUAL speed

// Microstep configuration
#define MICROSTEPS     16     // 1/16 microstepping

// StallGuard4 threshold (range: 0-255 for SG4)
#define SG4_THRESHOLD  50

// Current settings
#define RUN_CURRENT_mA  1200  // Run current in mA
#define HOLD_CURRENT_mA  800  // Hold current in mA
#define HOLD_MULTIPLIER 0.5   // Hold current as fraction of run current

// Over-temperature pre-warn threshold (Celsius)
#define OTPW_THRESHOLD 105.0

// Overvoltage threshold (Volts)
#define OV_THRESHOLD 30.0

// Timing for monitoring updates (milliseconds)
#define MONITOR_INTERVAL 500

// Forward declarations
void print_adc_readings();
void print_sg4_status();
void print_uart_status();
void print_drv_status();
void print_stealthchop_status();

// ============================================
// Select your TMC2240 configuration
// ============================================

// Option 1: Hardware UART (default, uncommented)
TMC2240Stepper driver(&SERIAL_PORT, R_SENSE);

// Option 2: Software UART (uncomment if needed)
// TMC2240Stepper driver(SW_RX, SW_TX, R_SENSE);

// Option 3: SPI mode (for comparison)
// TMC2240Stepper driver(CS_PIN, R_SENSE);
// TMC2240Stepper driver(CS_PIN, SW_MOSI, SW_MISO, SW_SCK);

// UART node configuration for daisy-chain
#define NODE_ADDRESS    0x00  // Node address (0-7)
#define SEND_DELAY      0x00  // Send delay (0-7)

bool stealthchop_enabled = true;
bool direction_forward = true;
uint32_t current_speed = DEFAULT_SPEED;

void setup() {
  // Initialize PC serial communication
  Serial.begin(PC_SERIAL_BAUD);
  while (!Serial) {
    delay(10);  // Wait for serial port to connect (needed for native USB)
  }

  Serial.println(F("\n========================================"));
  Serial.println(F("  TMC2240 UART Example"));
  Serial.println(F("========================================"));
  Serial.println(F("\nInitializing..."));

  // Configure GPIO pins
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);

  // Enable driver in hardware (LOW = enabled)
  digitalWrite(EN_PIN, LOW);

  // ============================================
  // Initialize UART Communication
  // ============================================

  Serial.println(F("\n--- UART Initialization ---"));

#if defined(SERIAL_PORT)
  // Initialize hardware UART to TMC2240
  SERIAL_PORT.begin(TMC_UART_BAUD);
  Serial.print(F("  Hardware UART initialized: "));
  Serial.println(TMC_UART_BAUD);
#else
  // For software UART, call beginSerial
  // driver.beginSerial(TMC_UART_BAUD);
  // Serial.print(F("  Software UART initialized: "));
  // Serial.println(TMC_UART_BAUD);
#endif

  // Initialize the TMC2240 driver
  driver.begin();

  // Test connection to verify communication
  Serial.print(F("  Connection test: "));
  uint8_t connection_status = driver.test_connection();
  if (connection_status == 0) {
    Serial.println(F("OK"));
  } else {
    Serial.print(F("FAILED (error code: "));
    Serial.print(connection_status);
    Serial.println(F(")"));
    Serial.println(F("  WARNING: Check UART connections!"));
  }

  // ============================================
  // UART Node Configuration (for daisy-chain)
  // ============================================

  // Configure node address and send delay for daisy-chain operation
  uint16_t nodeconf = (SEND_DELAY << 6) | (NODE_ADDRESS << 4);
  driver.NODECONF(nodeconf);
  Serial.print(F("  Node Address: "));
  Serial.println(NODE_ADDRESS);
  Serial.print(F("  Send Delay: "));
  Serial.println(SEND_DELAY);

  // ============================================
  // Basic Motor Configuration
  // ============================================

  // Enable driver in software (TOFF controls internal enable)
  driver.toff(4);  // Enable stepper driver (value 1-15)

  // Set RMS current
  driver.rms_current(RUN_CURRENT_mA, HOLD_MULTIPLIER);

  // Set microstepping
  driver.microsteps(MICROSTEPS);

  // Configure IHOLD/IRUN separately (alternative to rms_current)
  driver.ihold(10);        // Hold current scaling (0-31)
  driver.irun(16);         // Run current scaling (0-31)
  driver.iholddelay(1);    // Delay before switching to hold current

  Serial.println(F("\n--- Basic Configuration ---"));
  Serial.print(F("  Run Current: "));
  Serial.print(RUN_CURRENT_mA);
  Serial.println(F(" mA"));
  Serial.print(F("  Hold Current: "));
  Serial.print(HOLD_CURRENT_mA);
  Serial.println(F(" mA"));
  Serial.print(F("  Microsteps: 1/"));
  Serial.println(driver.microsteps());

  // ============================================
  // CHOPCONF Configuration
  // ============================================

  // Chopper configuration
  driver.TBL(2);           // Blank time (2 = 16ns, typical)
  driver.chm(false);       // Chopper mode: false = spreadCycle, true = fullstep
  driver.intpol(true);     // Interpolate to 256 microsteps
  driver.dedge(true);      // Double edge stepping
  driver.hstrt(3);         // Hysteresis start value
  driver.hend(-3);         // Hysteresis end value (can be negative)

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

  // Set StallGuard4 threshold (range: 0-255, higher = less sensitive)
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

  // ============================================
  // VACTUAL Speed Control
  // ============================================

  // Set initial speed using VACTUAL (UART-specific feature)
  // VACTUAL = 0 means use STEP input
  // VACTUAL != 0 sets speed via internal generator
  current_speed = DEFAULT_SPEED;
  driver.VACTUAL(current_speed);

  Serial.println(F("\n--- VACTUAL Speed Control (UART) ---"));
  Serial.print(F("  Speed (VACTUAL): "));
  Serial.println(current_speed);

  Serial.println(F("\nConfiguration complete!"));
  Serial.println(F("Available serial commands:"));
  Serial.println(F("  speed <value>  - Set VACTUAL speed (0 = stop)"));
  Serial.println(F("  forward         - Move forward (positive speed)"));
  Serial.println(F("  backward        - Move backward (negative speed)"));
  Serial.println(F("  status          - Print full status"));
  Serial.println(F("  temp            - Print temperature"));
  Serial.println(F("  voltage         - Print supply voltage"));
  Serial.println(F("  stealth         - Toggle stealthChop"));
  Serial.println(F("  spread          - Toggle spreadCycle"));
  Serial.println(F("  sg4 <value>     - Set StallGuard4 threshold"));
  Serial.println(F("  stop            - Stop motor"));
  Serial.println();
}

void loop() {
  // Handle serial commands
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("speed ")) {
      uint32_t speed = cmd.substring(6).toInt();
      current_speed = speed;
      driver.VACTUAL(speed);
      Serial.print(F("  Speed set to: "));
      Serial.println(speed);
    }
    else if (cmd == "forward") {
      direction_forward = true;
      current_speed = abs(current_speed);
      driver.VACTUAL(current_speed);
      Serial.println(F("  Direction: Forward"));
    }
    else if (cmd == "backward") {
      direction_forward = false;
      current_speed = -abs(current_speed);
      driver.VACTUAL(current_speed);
      Serial.println(F("  Direction: Backward"));
    }
    else if (cmd == "status") {
      Serial.println(F("\n--- Full Status ---"));
      print_adc_readings();
      print_drv_status();
      if (stealthchop_enabled) {
        print_stealthchop_status();
      }
      print_sg4_status();
      print_uart_status();
      Serial.println();
    }
    else if (cmd == "temp") {
      Serial.print(F("  Temperature: "));
      Serial.print(driver.get_chip_temperature(), 1);
      Serial.println(F(" °C"));
    }
    else if (cmd == "voltage") {
      Serial.print(F("  Supply Voltage: "));
      Serial.print(driver.get_vsupply_voltage(), 1);
      Serial.println(F(" V"));
    }
    else if (cmd == "stealth") {
      stealthchop_enabled = !stealthchop_enabled;
      driver.en_pwm_mode(stealthchop_enabled);
      if (stealthchop_enabled) {
        driver.pwm_autoscale(true);
        driver.pwm_autograd(true);
      }
      driver.push();
      Serial.print(F("  StealthChop: "));
      Serial.println(stealthchop_enabled ? F("Enabled") : F("Disabled"));
    }
    else if (cmd == "spread") {
      stealthchop_enabled = false;
      driver.en_pwm_mode(false);
      driver.en_spreadCycle(true);
      driver.push();
      Serial.println(F("  spreadCycle: Enabled"));
    }
    else if (cmd.startsWith("sg4 ")) {
      uint8_t sg4_val = cmd.substring(4).toInt();
      driver.sgt(sg4_val);
      driver.push();
      Serial.print(F("  SG4 threshold set to: "));
      Serial.println(sg4_val);
    }
    else if (cmd == "stop") {
      current_speed = 0;
      driver.VACTUAL(0);
      Serial.println(F("  Motor stopped"));
    }
    else if (!cmd.isEmpty()) {
      Serial.print(F("  Unknown command: "));
      Serial.println(cmd);
      Serial.println(F("  Type 'status' for help."));
    }
  }

  // Periodic monitoring
  static uint32_t last_monitor = 0;
  uint32_t now = millis();

  if (now - last_monitor >= MONITOR_INTERVAL) {
    last_monitor = now;

    print_adc_readings();
    print_drv_status();

    if (stealthchop_enabled) {
      print_stealthchop_status();
    }

    print_sg4_status();
    print_uart_status();

    Serial.print(F("  Speed (VACTUAL): "));
    Serial.print(current_speed);
    Serial.println();

    Serial.println();
  }

  // Small delay to prevent flooding
  delay(10);
}

/**
 * Print ADC-based readings (temperature, voltage)
 */
void print_adc_readings() {
  // Get chip temperature (uses internal ADC)
  float temp = driver.get_chip_temperature();
  Serial.print(F("  Temperature: "));
  Serial.print(temp, 1);
  Serial.println(F(" °C"));

  // Get supply voltage (uses internal ADC)
  float vsup = driver.get_vsupply_voltage();
  Serial.print(F("  Supply Voltage: "));
  Serial.print(vsup, 1);
  Serial.println(F(" V"));

  // Get AIN pin voltage (external ADC input)
  float ain = driver.get_ain_voltage();
  Serial.print(F("  AIN Voltage: "));
  Serial.print(ain, 3);
  Serial.println(F(" V"));

  // Check protection thresholds
  float otpw = driver.get_overtemp_prewarn_celsius();
  float ovth = driver.get_overvoltage_threshold_voltage();
  Serial.print(F("  OTPreWarn: "));
  Serial.print(otpw, 1);
  Serial.print(F(" °C | OVThreshold: "));
  Serial.print(ovth, 1);
  Serial.println(F(" V"));
}

/**
 * Print DRV_STATUS register flags
 */
void print_drv_status() {
  // Read DRV_STATUS register
  uint32_t drv_status = driver.DRV_STATUS();

  // Check individual flags
  bool stall = driver.stst();
  bool ot = driver.ot();
  bool otpw = driver.otpw();
  bool ola = driver.ola();
  bool olb = driver.olb();
  bool s2ga = driver.s2ga();
  bool s2gb = driver.s2gb();
  bool sg4 = driver.sg4();

  Serial.print(F("  Stall: "));
  Serial.print(stall ? F("Yes") : F("No"));
  Serial.print(F(" | Over-temp: "));
  Serial.print(ot ? F("Yes") : F("No"));
  Serial.print(F(" | OTPreWarn: "));
  Serial.print(otpw ? F("Yes") : F("No"));
  Serial.print(F(" | Open Load A: "));
  Serial.print(ola ? F("Yes") : F("No"));
  Serial.print(F(" | Open Load B: "));
  Serial.print(olb ? F("Yes") : F("No"));
  Serial.print(F(" | Short to Gnd A: "));
  Serial.print(s2ga ? F("Yes") : F("No"));
  Serial.print(F(" | Short to Gnd B: "));
  Serial.print(s2gb ? F("Yes") : F("No"));
  Serial.print(F(" | SG4: "));
  Serial.println(sg4 ? F("Yes") : F("No"));

  // Current scale actual
  uint16_t cs_actual = driver.CS_ACTUAL();
  Serial.print(F("  Current Scale: "));
  Serial.print(cs_actual);
  Serial.println(F("/256"));

  // Microstep counter
  uint16_t mscnt = driver.MSCNT();
  Serial.print(F("  Microstep Counter: "));
  Serial.println(mscnt);
}

/**
 * Print StealthChop/PWM status
 */
void print_stealthchop_status() {
  Serial.print(F("  StealthChop: "));
  Serial.print(driver.en_pwm_mode() ? F("Enabled") : F("Disabled"));

  // PWM scale values (read from PWM_SCALE register)
  uint8_t scale_sum = driver.pwm_scale_sum();
  int16_t scale_auto = driver.pwm_scale_auto();
  uint8_t ofs_auto = driver.pwm_ofs_auto();
  uint8_t grad_auto = driver.pwm_grad_auto();

  Serial.print(F(" | PWM Scale Sum: "));
  Serial.print(scale_sum);
  Serial.print(F(" | PWM Scale Auto: "));
  Serial.print(scale_auto);
  Serial.print(F(" | PWM Offset Auto: "));
  Serial.print(ofs_auto);
  Serial.print(F(" | PWM Gradient Auto: "));
  Serial.println(grad_auto);
}

/**
 * Print StallGuard4 status and result
 */
void print_sg4_status() {
  Serial.print(F("  SG4 Enabled: "));
  Serial.print(driver.sg4_en() ? F("Yes") : F("No"));

  // Read StallGuard4 result (SG4_RESULT register)
  uint16_t sg4_result = driver.sg4_result();
  Serial.print(F(" | StallGuard4 Result: "));
  Serial.print(sg4_result);

  // Read StallGuard threshold
  int8_t sgt = driver.sgt();
  Serial.print(F(" | StallGuard4 Threshold: "));
  Serial.println(sgt);
}

/**
 * Print UART-specific status information
 */
void print_uart_status() {
  // Read interface count (checks UART vs SPI)
  uint8_t ifcnt = driver.IFCNT();
  Serial.print(F("  Interface: "));
  Serial.println(ifcnt < 0x80 ? F("UART") : F("SPI"));

  // TX byte counter
  Serial.print(F("  Bytes Written: "));
  Serial.println(driver.bytesWritten);

  // CRC error flag
  Serial.print(F("  CRC Errors: "));
  Serial.println(driver.CRCerror ? "Yes" : "No");

  // Current VACTUAL value
  uint32_t vactual = driver.VACTUAL();
  Serial.print(F("  VACTUAL: "));
  Serial.println(vactual);
}