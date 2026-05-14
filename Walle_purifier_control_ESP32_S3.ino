/*

GitHub upload mark UV lamps and voltage sense
================================================================================
                          Wall-E UV Lamp Monitor System
                      LoRa-based Distributed Monitoring
================================================================================

Made by: Andani E. López Aréchar

SYSTEM DESCRIPTION:
  Wall-E is a distributed monitoring system for UV disinfectant lamps in
  production lines. The system operates at 110Vac and uses ESP32 microcontrollers
  with LoRa communication on 433MHz band.

LAMPS DESCRIPTION:
  - UV lamps are used for disinfection air in production lines
  - Model: Microbial Area Kleaner MAK-414: It seems to has 4 UV lamps per unit, works at 110Vac, 4.2A current in total (1.05A per lamp)
  - Each lamp has a current sensor (ACS712T-5A Hall-effect with voltage divisor) to monitor its operation
  - AC voltage sensor (ZMPT101B) monitors mains voltage presence/absence and RMS value


ARCHITECTURE:
  - Central Hub: Raspberry Pi 4 (receiver/coordinator)
  - Remote Nodes: Up to 255 ESP32 units (transmitters/responders)
  - Communication: LoRa point-to-point on 433MHz
  - Protocol: Request/Response with acknowledgment

MONITORING CAPABILITIES:
  1. Dual-mode Current Monitoring: AC (RMS) or DC (average) via ACS712T-5A (4 channels)
  2. AC Voltage monitoring (ZMPT101B) for mains presence detection and RMS voltage
  3. Multi-sensor support (4 current channels per node)
  4. Automatic measurements reporting on request with scaled 8-bit values

HARDWARE COMPONENTS:
  - ESP32-S3 WROOM DevKit microcontroller
  - SX1278 LoRa module (433MHz)
  - 4x ACS712T-5A Hall-effect current sensors with 1:2 voltage divisor (5.4V → 2.7V → 1.35V nominal)
  - AC voltage monitor (ZMPT101B with RMS conditioning)
  - Power supply (5.4V external for sensors, 5V/USB for ESP32, 3.3V for LoRa)
  - Industrial-grade enclosure

COMMUNICATION PROTOCOL:
  Request Packet (from Raspberry Pi):
    [NET_ID | MSG_REQ | TARGET_ID | REQ_CODE]
    - NET_ID: Network identifier (0xA5 for private network)
    - MSG_REQ: Message type = 0x10 (request)
    - TARGET_ID: 1-255 (device ID to query)
    - REQ_CODE: 0x01 (read sensor data)

  Response Packet (from ESP32):
    [NET_ID | MSG_RESP | DEVICE_ID | SEQ_LO | SEQ_HI | AC_V_SCALED | CURR1_SCALED | CURR2_SCALED | CURR3_SCALED | CURR4_SCALED]
    - NET_ID: Echo network ID (0xA5)
    - MSG_RESP: Message type = 0x90 (response)
    - DEVICE_ID: 1-255 (sender device ID)
    - SEQ_LO | SEQ_HI: 16-bit sequence number for tracking
    - AC_V_SCALED: AC voltage scaled 0-255 (maps 0.0-130.0 V RMS)
    - CURR1_SCALED: Current 1 scaled 0-255 (maps 0-2550 mA or 0-2.55A)
    - CURR2_SCALED: Current 2 scaled 0-255 (maps 0-2550 mA or 0-2.55A)
    - CURR3_SCALED: Current 3 scaled 0-255 (maps 0-2550 mA or 0-2.55A)
    - CURR4_SCALED: Current 4 scaled 0-255 (maps 0-2550 mA or 0-2.55A)

STATUS INFORMATION:
  Measurements are scaled to 8-bit values to fit in the 10-byte packet:
  - Voltage: 0-255 represents 0-130V RMS with resolution 0.51 V/step
  - Current: 0-255 represents 0-2550 mA (0-2.55A) with resolution 10 mA/step
  - Coordinator can reconstruct original values from scaled values
  - No status codes (0=OK/1=ALERT) in current implementation; all measurements sent directly

OPERATION FLOW:
  1. ESP32 initializes and enters listening mode
  2. Continuously reads sensor values (ADC pins) and applies filtering
  3. When request arrives, validates NET_ID and TARGET_ID
  4. If match found, scales current measurements to 8-bit format
  5. Builds and sends response packet back to coordinator
  6. Returns to listening mode

ADJUSTMENTS PER INSTALLATION:
  - TX_ID: Set to 1-255 for each device (line 149)
  - CURRENT_MEASUREMENT_AC: Set true for AC mode (RMS×5) or false for DC mode (line 265)
  - SIMULATE_MODE: Set true to test without sensors, false for real deployment (line 268)
  - CURRENT_THRESHOLD_MIN/MAX: For optional firmware-level filtering if needed (lines 277-278)
  - AC_VOLTAGE_THRESHOLD: Alert threshold if implementing local logic (line 279)
  - LoRa frequency: 433E6 (Asia), 866E6 (Europe), 915E6 (Americas) (line 259)

================================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ----------------------------------------------------------------------------------
// NOTA IMPORTANTE SOBRE PINES SPI EN ESP32-S3:
// El ESP32-S3 permite asignar cualquier función SPI (MOSI, MISO, SCK, CS) a casi
// cualquier GPIO mediante software. Por eso, en el pinout/datasheet solo aparecen
// los números de GPIO y no la función SPI fija. La asignación se realiza así:
//   SPI.begin(SCK, MISO, MOSI, SS);
// En este proyecto:
//   SCK  = GPIO12 (Pin 19, FSPI_CLK)
//   MISO = GPIO13 (Pin 18, FSPI_Q)
//   MOSI = GPIO11 (Pin 17, FSPI_D)
//   NSS  = GPIO18 (Pin 9)
//   RESET= GPIO14 (Pin 13)
//   DIO0 = GPIO26 (Pin 38)
// ----------------------------------------------------------------------------------
// Tabla de conexión SX1278 ↔ ESP32-S3-DevKitC-1:
// SX1278 Pin | ESP32 GPIO | ESP32 Pin# | Función en código
// -----------|------------|------------|------------------
// SCK        | GPIO12     | Pin 19     | SPI SCK
// MISO       | GPIO13     | Pin 18     | SPI MISO
// MOSI       | GPIO11     | Pin 17     | SPI MOSI
// NSS (CS)   | GPIO18     | Pin 9      | LORA_SS
// RESET      | GPIO14     | Pin 13     | LORA_RST
// DIO0       | GPIO46     | Pin 44     | LORA_DIO0
// 3.3V       | 3V3        | Pin 2/3    | Alimentación
// GND        | GND        | Pin 1/15/16| Tierra
// ----------------------------------------------------------------------------------

// ============================================================================
// HARDWARE PIN CONFIGURATION (ESP32-S3-WROOM-1 DevKitC-1)
// ============================================================================
// FULL PIN ALLOCATION SUMMARY
// ─────────────────────────────────────────────────────────────────────────
// GPIO  │ Role                          │ Type
// ──────┼───────────────────────────────┼────────────────────────────────
//   2   │ Onboard LED                   │ Digital output
//   4   │ Current sensor 1 (ACS712)     │ ADC1_CH3  (analog input)
//   5   │ Current sensor 2 (ACS712)     │ ADC1_CH4  (analog input)
//   6   │ Current sensor 3 (ACS712)     │ ADC1_CH5  (analog input)
//   7   │ Current sensor 4 (ACS712)     │ ADC1_CH6  (analog input)
//   8   │ I2C SDA  ← RESERVED           │ Wire SDA  (OLED SSD1306 + SEN0343)
//   9   │ I2C SCL  ← RESERVED           │ Wire SCL  (OLED SSD1306 + SEN0343)
//  11   │ LoRa MOSI                     │ SPI MOSI
//  12   │ LoRa SCK                      │ SPI SCK
//  13   │ LoRa MISO                     │ SPI MISO
//  14   │ LoRa RESET                    │ Digital output
//  15   │ AC voltage sensor (ZMPT101B)  │ ADC1_CH14 (analog input)
//  16   │ Tower relay GREEN             │ Digital output
//  17   │ Tower relay RED               │ Digital output
//  18   │ LoRa NSS (CS)                 │ SPI CS
//  35   │ L298N IN1 (Motor A dir+)      │ Digital output
//  36   │ L298N IN2 (Motor A dir-)      │ Digital output
//  37   │ L298N IN3 (Motor B dir+)      │ Digital output
//  38   │ L298N IN4 (Motor B dir-)      │ Digital output
//  39   │ L298N ENA (Motor A enable)    │ PWM output (LEDC)
//  40   │ L298N ENB (Motor B enable)    │ PWM output (LEDC)
//  46   │ LoRa DIO0                     │ Digital input (IRQ)
//  48   │ NeoPixel WS2812 RGB LED       │ Digital output
// ─────────────────────────────────────────────────────────────────────────
// Free GPIOs (not connected): 0,1,3,10,19,20,21,41-45,47
// Reserved system: 26-32 (internal flash SPI0), 43 (UART0 TX), 44 (UART0 RX),
//                  19-20 (USB D-/D+)
// ─────────────────────────────────────────────────────────────────────────

// ---- ADC Sensor Pins (ESP32-S3 ADC1 channels) ----
// Valid ADC1 pins on ESP32-S3: GPIO0-8, GPIO14-15
#define CURRENT_SENSOR1 4     // GPIO4 (ADC1_CH3) - ACS712T-5A Channel 1
#define CURRENT_SENSOR2 5     // GPIO5 (ADC1_CH4) - ACS712T-5A Channel 2
#define CURRENT_SENSOR3 6     // GPIO6 (ADC1_CH5) - ACS712T-5A Channel 3
#define CURRENT_SENSOR4 7     // GPIO7 (ADC1_CH6) - ACS712T-5A Channel 4
#define AC_POWER_PIN    15    // GPIO15 (ADC1_CH14) - AC voltage detector (ZMPT101B)

// ---- LoRa Module Pins (SX1278/RFM95) ----
#define LORA_SCK        12    // GPIO12 (Pin 19)
#define LORA_MISO       13    // GPIO13 (Pin 18)
#define LORA_MOSI       11    // GPIO11 (Pin 17)
#define LORA_SS         18    // GPIO18 (Pin 9)
#define LORA_RST        14    // GPIO14 (Pin 13)
#define LORA_DIO0       46    // GPIO46 (Pin 44)

// ---- LoRa RF profile (robust mode for noisy industrial environments) ----
#define LORA_FREQUENCY_HZ    433E6
#define LORA_SPREADING_FACTOR 10      // Higher sensitivity than SF7, still practical latency
#define LORA_BANDWIDTH_HZ    125E3
#define LORA_CODING_RATE     8        // CR 4/8 = strongest forward error correction
#define LORA_PREAMBLE_LEN    12       // Longer preamble improves packet detection
#define LORA_TX_POWER_DBM    17       // Safe high power for SX1278 modules

// ---- Indicator LED ----
#define LED_PIN         2     // Onboard LED (GPIO2) - indicates system running

// ---- Tower light relay outputs ----
// GPIO16 and GPIO17 are free digital GPIOs on the ESP32-S3-WROOM-1 DevKitC-1.
// The internal flash/PSRAM on WROOM-1 uses GPIO26-32 (SPI0), so 16 and 17 are safe.
// They don't interfere with ADC1 channels (GPIO0-8 used there), boot strapping, or USB.
//
// Wiring: connect each output to IN pin of your optocoupler relay board.
//   GREEN relay → K1 → turns ON the green tower light  (lamps OK)
//   RED   relay → K2 → turns ON the red  tower light  (lamp fault / no voltage)
//
// TOWER_RELAY_ACTIVE_HIGH:
//   true  = relay energizes when GPIO drives HIGH (standard optocoupler boards with
//           on-board flyback and INPUT tied to 3.3 V logic via 1k resistor).
//   false = relay energizes when GPIO drives LOW  (active-LOW boards, less common).
#define TOWER_RELAY_GREEN_PIN   16    // GPIO16 → relay K1 (green light)
#define TOWER_RELAY_RED_PIN     17    // GPIO17 → relay K2 (red light)
#define TOWER_RELAY_ACTIVE_HIGH false // Active-LOW relay board (LOW energizes coil)

// ---- I2C Bus – reserved for future OLED display and pressure sensor ----
// Both devices share the same I2C bus (Wire).  Call Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN)
// once in setup() when those peripherals are added.
//
// GPIO8 / GPIO9 are the ESP32-S3 Arduino-core default SDA/SCL pins and are free here.
//
// Device addresses (7-bit, as used by Arduino Wire):
//   OLED SSD1306 128×64 :  0x3C  (= 0x78 in 8-bit datasheet notation >> 1)
//   SEN0343 LWLP5000    :  0x00  (fixed address confirmed by DFRobot wiki / datasheet)
//
// NOTE ON SEN0343 ADDRESS 0x00:
//   The LWLP5000 chip uses I2C address 0x00 (confirmed in DFRobot official example).
//   This is an unusually low address – do NOT confuse it with the I2C general-call
//   address (also 0x00 in some specs).  The DFRobot_LWLP library handles it correctly.
#define I2C_SDA_PIN          8     // GPIO8  – Wire SDA (OLED + pressure sensor)
#define I2C_SCL_PIN          9     // GPIO9  – Wire SCL (OLED + pressure sensor)
#define OLED_I2C_ADDR        0x3C  // SSD1306 7-bit address  (datasheet shows 0x78 = 8-bit)
#define PRESSURE_I2C_ADDR    0x00  // SEN0343 LWLP5000 7-bit address (fixed, per DFRobot)

// ---- L298N H-Bridge Motor Driver ----
// ENA and ENB: leave the JUMPER installed on the L298N board.
//   This ties ENA/ENB permanently to the board's 5 V rail → always enabled → max power.
//   No GPIO is needed for enable; just control direction with IN1-IN4.
//
// Direction at startup (both motors forward, maximum duty cycle via hardware jumper):
//   Motor A: IN1=HIGH, IN2=LOW  → forward
//   Motor B: IN3=HIGH, IN4=LOW  → forward
//
// To reverse a motor swap the HIGH/LOW on its pair (IN1/IN2 or IN3/IN4).
// To brake a motor set both pins of a pair to the SAME level (both HIGH or both LOW).
//
// GPIO35-38 are safe on WROOM-1: not strapping pins, not internal-flash SPI, not USB.
#define L298N_IN1  35   // GPIO35 → L298N IN1  (Motor A +)
#define L298N_IN2  36   // GPIO36 → L298N IN2  (Motor A -)
#define L298N_IN3  37   // GPIO37 → L298N IN3  (Motor B +)
#define L298N_IN4  38   // GPIO38 → L298N IN4  (Motor B -)
#define L298N_ENA  39   // GPIO39 → L298N ENA  (Motor A enable – PWM)
#define L298N_ENB  40   // GPIO40 → L298N ENB  (Motor B enable – PWM)

// PWM configuration for ENA/ENB (100% duty = máxima potencia constante)
// Frecuencia 1 kHz: suficiente para el L298N, sin ruido audible apreciable.
// Resolución 8-bit: duty 0-255, 255 = 100%.
#define L298N_PWM_FREQ  1000   // Hz
#define L298N_PWM_RES   8      // bits (0-255)
#define L298N_PWM_DUTY  255    // 100% – máxima potencia

// WS2812 RGB LED (NeoPixel) configuration
#define NEOPIXEL_PIN    48    // GPIO48 for WS2812 addressable RGB LED
#define NEOPIXEL_COUNT  1     // Single LED on most DevKits
Adafruit_NeoPixel neopixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Color constants (GRB format for WS2812)
#define COLOR_OFF       neopixel.Color(0, 0, 0)
#define COLOR_RED       neopixel.Color(0, 255, 0)
#define COLOR_GREEN     neopixel.Color(255, 0, 0)
#define COLOR_BLUE      neopixel.Color(0, 0, 255)
#define COLOR_YELLOW    neopixel.Color(255, 200, 0)
#define COLOR_CYAN      neopixel.Color(0, 255, 255)
#define COLOR_WHITE     neopixel.Color(255, 255, 255)
#define COLOR_PURPLE    neopixel.Color(128, 0, 128)

// OLED SSD1306 128x64 display object (I2C via GPIO8=SDA, GPIO9=SCL)
Adafruit_SSD1306 display(128, 64, &Wire, -1);


// ============================================================================
// DEVICE CONFIGURATION
// ============================================================================

/*
 * **CRITICAL: Set TX_ID to 1-255, UNIQUE for each device**
 * 
 * Device 1: TX_ID = 1
 * Device 2: TX_ID = 2
 * Device 3: TX_ID = 3
 * ...
 * Device 11: TX_ID = 11
 *
 * Each ESP32 must have a different TX_ID to identify itself to the coordinator
 */
#define TX_ID           1     // CHANGE THIS FOR EACH DEVICE (1-255) !!!

#if (TX_ID < 1) || (TX_ID > 255)
#error "TX_ID must be between 1 and 255"
#endif

// ============================================================================
// PROTOCOL CONSTANTS
// ============================================================================

#define NET_ID          0xA5  // Network identifier (private network marker)
#define MSG_REQ         0x10  // Message type: Request from coordinator
#define MSG_RESP        0x90  // Message type: Response from node
#define REQ_READ_DATA   0x01  // Request code: Read sensor data

// ============================================================================
// SENSOR CALIBRATION & CONVERSION
// ============================================================================
/*
 * ACS712T-5A Current Sensor Calibration (Hall-effect sensor with voltage divisor):
 *   - Power supply: 5.4V (measured external supply for sensors)
 *   - Sensor sensitivity: 185 mV/A (at VCC operation)
 *   - DC Offset at no load: 2.7V (VCC/2 = 5.4V/2)
 *   - With voltage divisor 1:2 (860 ohm / 860 ohm): 2.7V → 1.35V theoretical (1.55V measured due to tolerance)
 *   - Full scale range: 0-5A mapped to ±462.5mV from offset
 *   - Sensitivity after divisor: 185mV/A ÷ 2 = 92.5 mV/A on ESP32 ADC
 *   - ESP32-S3 ADC: 12-bit (0-4095 = 0-3.3V)
 *   - ADC resolution: 3.3V / 4095 = 0.8056 mV per step
 *   - Safe input range with 5.4V supply: ~1.1V to 2.0V (well within 3.3V ADC max)
 *   
 * ZMPT101B AC Voltage Sensor Calibration (with 1:2 divider 860 ohm / 860 ohm):
 *   - Supply: 5.43V (measured)
 *   - Input: 118V RMS AC mains (measured with multimeter)
 *   - Measured RMS at sensor output BEFORE divider: 0.212V RMS
 *   - Divider 1:2 -> RMS at ADC AFTER divider: 0.106V RMS
 *   - Conversion ratio: 118V RMS / 0.106V RMS = 1113 V/V
 *   - DC offset (before divider): 2.716V (AC disconnected)
 *   - DC offset (after divider): 1.358V
 *   - Alert threshold: < 100V RMS
 *   
 * MEASUREMENT PROCESS FOR ACS712T:
 *   1. Sample ADC multiple times for averaging
 *   2. Calculate average voltage (or RMS for AC mode)
 *   3. Subtract DC offset (1.55V calibrated value)
 *   4. Convert voltage difference to current using sensitivity (92.5mV/A)
 *   5. Filter and apply noise floor
 *   
 * Measurement process for ZMPT101B (AC sensor):
 *   1. Sample 30 times (~1-2ms, appropriate for 50/60Hz AC)
 *   2. Remove DC offset from each sample
 *   3. Calculate RMS: sqrt(sum of squares / number of samples)
 *   4. Convert RMS to actual measurement (voltage in V)
 */

#define ADC_SAMPLES 200       // Number of samples per RMS calculation
#define RMS_ITERATIONS 5      // Number of RMS calculations to average (better noise rejection)

// ACS712T-5A current sensor calibration with voltage divisor
// Real installation specs:
//   - External power supply: 5.4V (measured)
//   - ACS712T sensitivity: 185 mV/A at VCC
//   - DC Offset at zero current: VCC/2 = 5.4V/2 = 2.7V
// With voltage divisor 1:2 (resistive divider to match ESP32 ADC 3.3V max):
//   - Theoretical offset after divisor: 2.7V / 2 = 1.35V
//   - ACTUAL measured offset: 1.55V (calibrated from real circuit)
//   - Difference (0.20V) caused by resistor tolerances (±5% typical)
//   - Sensitivity after divisor: 185mV/A / 2 = 92.5 mV/A
// 
// CALIBRATION NOTES:
//   - Always use measured offset value for accurate zero-current reference
//   - With 4.96V supply and divisor, safe ADC range: ~0.8V to 2.3V (well within 3.3V max)
//   - Maximum measurable current: 5A → voltage swing: ±462.5mV from offset
//   - CALIBRATION: Offset varies with supply voltage
//     * 5.4V supply → 1.55V offset (old calibration)
//     * 4.96V supply → 1.245V offset (average of measured values)
//       Measured values: S1=1.254V, S2=1.256V, S3=1.229V, S4=1.263V (avg=1.245V)
//   - SOFTWARE CORRECTION: After RMS calculation, subtract 40mA baseline offset
//     This compensates for residual readings (30-50mA observed with no load)
//     caused by ADC noise, sensor variation, and signal conditioning artifacts.
const float ACS712_DC_OFFSET_V = 1.245f;       // DC offset voltage with divisor (CALIBRATED at 4.96V supply)
const float ACS712_SENSITIVITY_mVpA = 92.5f;   // Sensitivity in mV/A (185mV/A ÷ 2 from divisor)
const float ACS712_SENSITIVITY_VpA = ACS712_SENSITIVITY_mVpA / 1000.0f;  // Convert to V/A
const float ACS712_MAX_CURRENT_A = 5.0f;       // Maximum measurable current (5A)
const float ACS712_BASELINE_CORRECTION_A = 0.000f;  // Zero-calibration baseline (no global current trim)

// ZMPT101B Voltage Sensor Calibration Constants
// =============================================
// VOLTAGE_RATIO: Conversion factor from ADC millivolts to actual mains voltage
//   - Measured mains RMS: 119.8V (with calibrated multimeter)
//   - ADC reads: 102mV RMS (after 1:2 voltage divider)
//   - Ratio: 119.8V / 0.102V = 1175 V/V
#define VOLTAGE_RATIO 1175

// VOLTAGE_CAL_FACTOR: Final calibration multiplier (configurable)
//   - Keep at 1.000f to preserve current real-site calibration (~122Vrms)
//   - Optional normalization to nominal 127Vrms: 127.0 / 122.0 = 1.041f
//   - Effective voltage = computed_voltage * VOLTAGE_CAL_FACTOR
//
// QUICK VOLTAGE CALIBRATION GUIDE:
//   1) Measure real mains with multimeter (V_real).
//   2) Read firmware value in serial/HMI (V_fw).
//   3) Update factor using:
//        VOLTAGE_CAL_FACTOR_NEW = VOLTAGE_CAL_FACTOR_OLD * (V_real / V_fw)
//   4) Reflash and verify with 10-20 samples (avoid single-sample tuning).
//
// Example:
//   If firmware shows 113V and multimeter shows 118V:
//   factor_new = factor_old * (118/113) = factor_old * 1.044
#define VOLTAGE_CAL_FACTOR 1.000f

// ZMPT_OFFSET_mV: Systematic offset correction for ADC readings
//   Root Cause: The ESP32 ADC and signal conditioning have a ~7mV systematic bias
//   when measuring the ZMPT101B AC output. This was discovered through calibration:
//   - Multimeter measured ADC pin: 102mV RMS (true value)
//   - ESP32 code calculated: 109mV RMS (7mV too high)
//   - After correction: 109 - 7 = 102mV → 102 × 1175 = 119.8V (correct!)
//   - Without this correction, voltage would read 8V too high (±5.2% error)
#define ZMPT_OFFSET_mV 7.0f
#define CURRENT_FLOOR_A 0.0   // Zero-calibration baseline (no noise floor clamp)

// Enable/disable multi-sensor reading: true = CURR1 only, false = all 4 sensors
#define USE_ONLY_SENSOR1 false

// Production wiring mode: only sensors 1 and 3 are active.
// Sensors 2 and 4 are physically mounted but intentionally unused.
#define USE_SENSORS_1_AND_3_ONLY true

// CURRENT MEASUREMENT MODE: Choose between AC and DC measurement
// Set to true for AC measurement (RMS), false for DC measurement (average)
#define CURRENT_MEASUREMENT_AC true

// Per-channel current gain trim (fine calibration).
// S3 is boosted based on observed stable delta in serial logs.
//
// QUICK CURRENT CALIBRATION GUIDE (S1/S3):
//   Use the serial line:
//     [CAL S1-S3] S1=...mA | S3=...mA | DELTA=...mA
//
//   A) Match absolute value to multimeter (same load on both channels):
//      gain_new = gain_old * (I_real / I_fw)
//
//   B) Match channels to each other (reduce DELTA):
//      If S3 < S1 consistently -> increase CURRENT_GAIN_CH3
//      If S3 > S1 consistently -> decrease CURRENT_GAIN_CH3
//
// Practical tuning order:
//   1) First set CH1 to multimeter reference.
//   2) Then tune CH3 to match CH1.
//   3) Re-check against multimeter and do one final small correction.
//
// Tip:
//   Tune with averaged values over ~20-60s, not single lines.
#define CURRENT_GAIN_CH1 1.000f
#define CURRENT_GAIN_CH2 1.000f
#define CURRENT_GAIN_CH3 1.000f
#define CURRENT_GAIN_CH4 1.000f

// TEST/SIMULATION MODE: Set to true to simulate sensor values for communication testing
// Set to false to use real sensor readings from ACS712T + ZMPT101B
#define SIMULATE_MODE true

// FORCE CURRENT TEST: Overrides ADC readings with a fixed value (in Amperes).
// Useful to verify relay/OLED logic without real sensors.
// Comment out this line to restore real ADC readings.
// #define FORCE_CURRENT_TEST_A  0.400f   // 400 mA forzado – descomentar para prueba

// Simulated sensor values – fijos en 400mA para prueba
float sim_current1_A = 0.400f;
float sim_current2_A = 0.400f;
float sim_current3_A = 0.400f;
float sim_current4_A = 0.400f;
float sim_voltage_V = 125.0;
static unsigned long lastSimUpdate = 0;

// Current sensor readings (RMS values in Amperes)
float current_sensor1_A = 0.0;
float current_sensor2_A = 0.0;
float current_sensor3_A = 0.0;
float current_sensor4_A = 0.0;

// Moving average filter for smoothing (10-sample average)
#define FILTER_SAMPLES 10
float filter_curr1[FILTER_SAMPLES] = {0};
float filter_curr2[FILTER_SAMPLES] = {0};
float filter_curr3[FILTER_SAMPLES] = {0};
float filter_curr4[FILTER_SAMPLES] = {0};
uint8_t filter_index1 = 0;
uint8_t filter_index2 = 0;
uint8_t filter_index3 = 0;
uint8_t filter_index4 = 0;

// DEBUG variables for diagnostics
float debug_raw_adc_avg = 0;
float debug_rms_curr1 = 0;
float debug_min_mV = 0;
float debug_max_mV = 0;
float AC_voltage_V = 0.0;     // AC voltage in Volts RMS
float AC_voltage_tx_V = 0.0;  // Voltage sent to coordinator (3-second moving mean)

// Telemetry scaling max for voltage packet encoding (must match coordinator decode)
const float VOLTAGE_SCALE_MAX_V = 130.0f;

// 3-second voltage averaging window for transmitted value
#define VOLTAGE_TX_AVG_WINDOW_MS 3000UL
unsigned long voltageAvgWindowStartMs = 0;
float voltageAvgAccumV = 0.0;
uint16_t voltageAvgSampleCount = 0;


// ============================================================================
// MEASUREMENT THRESHOLDS (Optional - for coordinator-side filtering)
// ============================================================================
/*
 * These thresholds are available for future firmware-level filtering/validation.
 * Currently they are stored but NOT used in this implementation.
 * The coordinator receives all measurements and handles filtering logic.
 *
 * CURRENT_THRESHOLD_MIN / CURRENT_THRESHOLD_MAX (in Amperes):
 *   - Define acceptable current range for each lamp
 *   - For UV lamps: typical range 0.5A to 1.2A per lamp
 *   - Can be used on ESP32 or coordinator depending on requirements
 *
 * AC_VOLTAGE_THRESHOLD (in Volts RMS):
 *   - Minimum acceptable mains voltage for safe operation
 *   - Below this value indicates potential power supply issue
 *   - Typical range: 95-110V RMS
 *
 * DEBUG / CALIBRATION:
 * 1. Open Serial Monitor (Tools → Serial Monitor, 115200 baud)
 * 2. Monitor "Current: CURR1=X.XXA ... | AC Voltage: XXX.XV RMS" output
 * 3. Verify measurements are in expected range with known loads
 * 4. Use these thresholds if implementing local validation
 */

float CURRENT_THRESHOLD_MIN = 0.5;   // Minimum acceptable current (Amperes)
float CURRENT_THRESHOLD_MAX = 1.2;   // Maximum acceptable current (Amperes)
float AC_VOLTAGE_THRESHOLD = 100.0;  // Minimum acceptable voltage (Volts RMS)

// Tower state thresholds
const float TOWER_CURRENT_THRESHOLD_A = 0.001f;  // >0 mA – cualquier señal = OK (sensor dañado)
const float TOWER_VOLTAGE_THRESHOLD_V = 100.0f;  // 100 V

enum TowerState {
  TOWER_STATE_RED = 0,
  TOWER_STATE_GREEN = 1
};

TowerState currentTowerState = TOWER_STATE_RED;

// ============================================================================
// LORA WATCHDOG CONFIGURATION
// ============================================================================
/*
 * The SX1278 LoRa module can occasionally enter an unresponsive state where
 * it stops receiving packets. This watchdog system monitors LoRa activity and
 * automatically resets the module if it becomes unresponsive.
 * 
 * WATCHDOG STRATEGY:
 * 1. Track timestamp of last LoRa activity (packet received OR parsePacket() call)
 * 2. Periodically check if too much time has passed without activity
 * 3. If timeout exceeded, perform hardware reset and reinitialize LoRa module
 * 4. LED turns RED during reset, returns to normal operation after recovery
 * 
 * TIMEOUT CONFIGURATION:
 * - LORA_WATCHDOG_TIMEOUT_MS: Time without activity before triggering reset
 *   * Default: 20000ms (20 seconds)
 *   * Adjust based on coordinator polling frequency
 *   * Should be ~3-5x the expected max time between requests
 *   * IMPORTANT: Must be LESS than Raspberry Pi's stale timeout (120s)
 *     to allow automatic recovery before coordinator raises alarm
 * 
 * ACTIVITY DETECTION:
 * - Updated in checkLoRaPackets() on every call (even if no packet)
 * - This ensures we detect both "no packets" and "module frozen" conditions
 * - A frozen module won't respond to parsePacket(), causing timeout
 */

#define LORA_WATCHDOG_TIMEOUT_MS 20000  // 20 seconds without activity = frozen
#define LORA_RESET_RETRY_MAX 3          // Max reset attempts before halting

unsigned long lastLoRaActivity = 0;     // Timestamp of last LoRa activity (millis)
uint8_t loraResetCount = 0;             // Counter for LoRa reset events (diagnostic)

// ============================================================================
// BITMAP – Purifier OK (Bimbo) 128x64
// ============================================================================
const unsigned char bmp_purifier_ok [] PROGMEM = {
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0xfc, 0x1f, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xff, 0x07, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1f, 0xff, 0xc1, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xf0, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0x80, 0x00, 0x7f, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0xe0, 0x00, 0x07, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0xf9, 0xff, 0xc0, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xff, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0xff, 0x3f, 0xfc, 0x7f,
  0xff, 0x07, 0xff, 0xff, 0xe3, 0x80, 0x7f, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0xff, 0xdf, 0xff, 0x3f,
  0xfe, 0x01, 0xff, 0xff, 0xe3, 0x80, 0x7f, 0xff, 0xfc, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f,
  0xfe, 0x00, 0xff, 0xff, 0xff, 0x0f, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf,
  0xfe, 0x18, 0x8e, 0x32, 0x23, 0x04, 0x70, 0x79, 0x1e, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf,
  0xfe, 0x18, 0x8e, 0x30, 0x22, 0x00, 0x60, 0x38, 0x1e, 0x3f, 0x80, 0x3f, 0xff, 0xff, 0xff, 0xe7,
  0xfe, 0x10, 0x8e, 0x30, 0x23, 0x0c, 0x63, 0x18, 0x3f, 0x3f, 0xf7, 0xc7, 0xff, 0xff, 0xff, 0xe7,
  0xfe, 0x01, 0x8e, 0x31, 0xe3, 0x1c, 0x60, 0x18, 0xff, 0x1f, 0xf7, 0xf9, 0xff, 0xff, 0xff, 0xe7,
  0xfe, 0x03, 0x8e, 0x31, 0xe3, 0x1c, 0x60, 0x18, 0xff, 0x8f, 0xfe, 0x0e, 0x3f, 0xff, 0xff, 0xe7,
  0xfe, 0x1f, 0x8e, 0x31, 0xe3, 0x1c, 0x60, 0x18, 0xff, 0x87, 0xee, 0x0f, 0xc7, 0xff, 0xff, 0xe7,
  0xfe, 0x1f, 0x84, 0x31, 0xe3, 0x1c, 0x63, 0xf8, 0xff, 0xc7, 0xee, 0x0f, 0xf0, 0xfe, 0x7f, 0xcf,
  0xfe, 0x1f, 0xc0, 0x31, 0xe3, 0x1c, 0x60, 0x18, 0xff, 0xe3, 0xec, 0x0f, 0xff, 0x03, 0xff, 0xcf,
  0xfe, 0x1f, 0xc0, 0x31, 0xe3, 0x1c, 0x70, 0x18, 0xff, 0xf0, 0xec, 0x0f, 0xff, 0xdf, 0xff, 0x8f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x3c, 0x0f, 0xff, 0xd4, 0x1f, 0x1f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe3, 0xdc, 0xff, 0xff, 0xdf, 0xee, 0x3f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xcf, 0xdf, 0xff, 0xff, 0xbf, 0xf4, 0x7f,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x8f, 0xdc, 0x03, 0xff, 0x7f, 0xf8, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9c, 0x03, 0xf8, 0x1f, 0x78, 0x79, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1b, 0xdf, 0xff, 0xe0, 0x7f, 0xb9, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1b, 0xff, 0xfc, 0x1f, 0xef, 0xb9, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x1b, 0xa3, 0xff, 0xde, 0xef, 0xb8, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9b, 0x9f, 0xff, 0xff, 0xef, 0xbc, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9d, 0x7f, 0xff, 0xff, 0xf7, 0x7c, 0xff,
  0xff, 0xff, 0xe0, 0xf9, 0xe3, 0xc7, 0x8f, 0x1f, 0xff, 0x9f, 0x7f, 0xff, 0xff, 0xf8, 0xf9, 0xff,
  0xff, 0xff, 0x80, 0x78, 0xc3, 0x83, 0x87, 0x1f, 0xff, 0xcf, 0x7f, 0xfe, 0xff, 0xff, 0xf9, 0xff,
  0xff, 0xff, 0x00, 0x38, 0x87, 0x83, 0xc6, 0x3f, 0xff, 0xc3, 0x77, 0xfc, 0x3f, 0xbf, 0xf1, 0xff,
  0xff, 0xff, 0x1e, 0x38, 0x8f, 0x83, 0xc2, 0x3f, 0xff, 0xe0, 0x67, 0xfc, 0x3f, 0xcf, 0xc3, 0xff,
  0xff, 0xff, 0x1e, 0x18, 0x1f, 0x11, 0xe0, 0x7f, 0xff, 0xf8, 0x4b, 0xfa, 0x9f, 0xf0, 0x07, 0xff,
  0xff, 0xfe, 0x1f, 0x18, 0x1f, 0x11, 0xe0, 0x7f, 0xff, 0xfe, 0x49, 0xf8, 0x9f, 0xf8, 0x1f, 0xff,
  0xff, 0xfe, 0x1f, 0x18, 0x1f, 0x18, 0xf0, 0xff, 0xff, 0xfe, 0x41, 0xfa, 0x1f, 0xf9, 0xff, 0xff,
  0xff, 0xfe, 0x1f, 0x18, 0x0e, 0x00, 0xf0, 0xff, 0xff, 0xfe, 0x51, 0xf8, 0x1f, 0xf9, 0xff, 0xff,
  0xff, 0xff, 0x1e, 0x18, 0x8e, 0x00, 0xf8, 0xff, 0xff, 0xfe, 0x63, 0xbc, 0x3f, 0xf9, 0xff, 0xff,
  0xff, 0xff, 0x0c, 0x38, 0x86, 0x00, 0x78, 0xff, 0xff, 0xfc, 0x7c, 0x3f, 0xff, 0xf8, 0xff, 0xff,
  0xff, 0xff, 0x80, 0x38, 0xc0, 0x3c, 0x78, 0xff, 0xff, 0xfc, 0xf0, 0x3f, 0xff, 0xfc, 0xff, 0xff,
  0xff, 0xff, 0xc0, 0x78, 0xe0, 0x7c, 0x78, 0xff, 0xff, 0xfc, 0xf4, 0x7f, 0xff, 0xfc, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf6, 0xfe, 0x3f, 0xf8, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfe, 0xff, 0xff, 0xf8, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xf8, 0x7b, 0xff, 0xf9, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0xfe, 0x03, 0xff, 0xf1, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0x03, 0xff, 0xe3, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0x01, 0xff, 0xc7, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x85, 0xff, 0x8f, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9f, 0x4b, 0xfe, 0x1f, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc7, 0xb7, 0xf8, 0x3f, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe1, 0xff, 0x80, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x03, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x1f, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

// ============================================================================
// OLED DISPLAY FUNCTION
// ============================================================================
/*
 * Updates the OLED screen based on the current tower state:
 *   GREEN → bitmap Bimbo (purifier OK)
 *   RED   → "PURIFIER" / "ERROR" (texto grande)
 */
void updateOLED() {
  display.clearDisplay();

  if (currentTowerState == TOWER_STATE_GREEN) {
    display.drawBitmap(0, 0, bmp_purifier_ok, 128, 64, SSD1306_WHITE);
  } else {
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(16, 8);
    display.print("PURIFIER");
    display.setTextSize(3);
    display.setCursor(19, 36);
    display.print("ERROR");
  }

  display.display();
}

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

uint16_t seq = 0;             // Response sequence counter (incremented each reply)

void setTowerRelayState(TowerState state) {
  const int relayOnLevel = TOWER_RELAY_ACTIVE_HIGH ? HIGH : LOW;
  const int relayOffLevel = TOWER_RELAY_ACTIVE_HIGH ? LOW : HIGH;

  if (state == TOWER_STATE_GREEN) {
    digitalWrite(TOWER_RELAY_GREEN_PIN, relayOnLevel);
    digitalWrite(TOWER_RELAY_RED_PIN, relayOffLevel);
  } else {
    digitalWrite(TOWER_RELAY_GREEN_PIN, relayOffLevel);
    digitalWrite(TOWER_RELAY_RED_PIN, relayOnLevel);
  }
}

bool areActiveCurrentsAboveTowerThreshold() {
  if (current_sensor1_A < TOWER_CURRENT_THRESHOLD_A) {
    return false;
  }

  if (USE_ONLY_SENSOR1) {
    return true;
  }

  if (USE_SENSORS_1_AND_3_ONLY) {
    return current_sensor3_A >= TOWER_CURRENT_THRESHOLD_A;
  }

  return (current_sensor2_A >= TOWER_CURRENT_THRESHOLD_A) &&
         (current_sensor3_A >= TOWER_CURRENT_THRESHOLD_A) &&
         (current_sensor4_A >= TOWER_CURRENT_THRESHOLD_A);
}

// ============================================================================
// SETUP FUNCTION - Initialization (runs once at power-on/reset)
// ============================================================================

void setup() {
  // Initialize LED for visual feedback
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize tower relays (default to RED on boot as fail-safe)
  pinMode(TOWER_RELAY_GREEN_PIN, OUTPUT);
  pinMode(TOWER_RELAY_RED_PIN, OUTPUT);
  setTowerRelayState(TOWER_STATE_RED);

  // L298N motor driver – pines reservados, sin activación por ahora
  // pinMode(L298N_IN1, OUTPUT);
  // pinMode(L298N_IN2, OUTPUT);
  // pinMode(L298N_IN3, OUTPUT);
  // pinMode(L298N_IN4, OUTPUT);
  // digitalWrite(L298N_IN1, HIGH);
  // digitalWrite(L298N_IN2, LOW);
  // digitalWrite(L298N_IN3, HIGH);
  // digitalWrite(L298N_IN4, LOW);
  // ledcAttach(L298N_ENA, L298N_PWM_FREQ, L298N_PWM_RES);
  // ledcWrite(L298N_ENA, L298N_PWM_DUTY);
  // ledcAttach(L298N_ENB, L298N_PWM_FREQ, L298N_PWM_RES);
  // ledcWrite(L298N_ENB, L298N_PWM_DUTY);

  // Initialize I2C bus and OLED display
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("WARNING: OLED SSD1306 not found – display disabled");
  } else {
    display.clearDisplay();
    display.display();
  }

  // Initialize NeoPixel early (used for error/status indicators)
  neopixel.begin();
  neopixel.setPixelColor(0, COLOR_OFF);
  neopixel.show();

  // Initialize UART serial for debugging output
  Serial.begin(115200);
  // NOTE: while (!Serial); is commented out because:
  //   - It blocks startup if no USB/Serial Monitor is connected
  //   - Our system must run autonomously without PC connection
  //   - We use delay(500) instead to give Serial time to stabilize if available
  delay(500);

  // Calibrate ADC for accurate millivolt readings
  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_SENSOR1, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR2, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR3, ADC_11db);
  analogSetPinAttenuation(CURRENT_SENSOR4, ADC_11db);
  analogSetPinAttenuation(AC_POWER_PIN, ADC_11db);
  
  // Print startup banner
  Serial.println("\n\n========================================");
  Serial.println("   Wall-E UV Monitor - Device " + String(TX_ID));
  Serial.println("========================================");
  Serial.println("Initializing LoRa module...");

  // Configure LoRa module SPI pins
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Initialize LoRa module with 433 MHz frequency
  // Using standard LoRa parameters for balanced range/speed
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    Serial.println("ERROR: LoRa initialization failed!");
    Serial.println("Possible causes:");
    Serial.println("  - Module not physically connected");
    Serial.println("  - Wrong pin configuration");
    Serial.println("  - SPI bus malfunction");
    neopixel.setPixelColor(0, COLOR_RED);
    neopixel.show();
  }

  // Configure LoRa physical layer parameters
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.setSyncWord(0x21);               // Sync word: 0x21 (private network)
  LoRa.enableCrc();                     // Enable CRC error checking
  LoRa.receive();                        // Ensure radio stays in RX mode

  // Print initialization complete message
  Serial.println("✓ LoRa initialized successfully!");
  Serial.println("  Frequency: 433 MHz");
  Serial.println("  Spreading Factor: 10");
  Serial.println("  Bandwidth: 125 kHz");
  Serial.println("  Coding Rate: 4/8");
  Serial.println("  Preamble Length: 12");
  Serial.println("  TX Power: 17 dBm");
  Serial.println("  Waiting for requests from coordinator...");
  Serial.println("========================================\n");

  // Standby: Azul
  neopixel.setPixelColor(0, COLOR_BLUE);
  neopixel.show();
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize LoRa watchdog timer
  lastLoRaActivity = millis();
}

// ============================================================================
// LORA MODULE RESET FUNCTION
// ============================================================================
/*
 * Performs a complete hardware reset and re-initialization of the LoRa module.
 * Called automatically by watchdog when module becomes unresponsive.
 * 
 * RESET PROCEDURE:
 * 1. Visual indication (RED LED)
 * 2. Hardware reset via RESET pin (LOW → delay → HIGH)
 * 3. Re-initialize SPI bus
 * 4. Re-configure LoRa module with all parameters
 * 5. Return to RX mode
 * 6. Update activity timestamp
 * 
 * RETURNS: true if reset successful, false if initialization failed
 */
bool resetLoRaModule() {
  loraResetCount++;
  
  Serial.println("\n⚠️  \033[1;31mLoRa WATCHDOG TRIGGERED\033[0m");
  Serial.printf("No activity detected for %d seconds\n", LORA_WATCHDOG_TIMEOUT_MS / 1000);
  Serial.printf("Attempting LoRa module reset (attempt #%d)...\n", loraResetCount);
  
  // Visual indication: RED = resetting
  neopixel.setPixelColor(0, COLOR_RED);
  neopixel.show();
  
  // Step 1: Hardware reset via RESET pin
  Serial.println("  [1/5] Performing hardware reset...");
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW);
  delay(100);
  digitalWrite(LORA_RST, HIGH);
  delay(100);
  
  // Step 2: Re-initialize SPI bus
  Serial.println("  [2/5] Re-initializing SPI bus...");
  SPI.end();
  delay(50);
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  // Step 3: Initialize LoRa module
  Serial.println("  [3/5] Initializing LoRa module...");
  if (!LoRa.begin(LORA_FREQUENCY_HZ)) {
    Serial.println("  \033[1;31m✗ FAILED: LoRa.begin() returned false\033[0m");
    Serial.println("  Possible causes:");
    Serial.println("    - Module hardware failure");
    Serial.println("    - Loose connection");
    Serial.println("    - SPI bus issue");
    return false;
  }
  
  // Step 4: Re-configure LoRa parameters
  Serial.println("  [4/5] Configuring LoRa parameters...");
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH_HZ);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LEN);
  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.setSyncWord(0x21);
  LoRa.enableCrc();
  
  // Step 5: Enter RX mode
  Serial.println("  [5/5] Entering receive mode...");
  LoRa.receive();
  
  // Reset successful
  Serial.println("\033[1;32m✓ LoRa module reset SUCCESSFUL\033[0m");
  Serial.printf("Total resets since boot: %d\n\n", loraResetCount);
  
  // Update activity timestamp
  lastLoRaActivity = millis();
  
  // Return to standby color
  neopixel.setPixelColor(0, COLOR_BLUE);
  neopixel.show();
  
  return true;
}

// ============================================================================
// HELPER FUNCTION - Moving Average Filter
// ============================================================================
/*
 * Applies a moving average low-pass filter to smooth noisy sensor readings.
 * Reduces oscillations without significantly delaying response.
 */
float applyMovingAverage(float newValue, float buffer[FILTER_SAMPLES], uint8_t &index) {
  // Store new value in circular buffer
  buffer[index] = newValue;
  index = (index + 1) % FILTER_SAMPLES;
  
  // Calculate average
  float sum = 0;
  for (uint8_t i = 0; i < FILTER_SAMPLES; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SAMPLES;
}


// ============================================================================
// HELPER FUNCTION - Single RMS Calculation (internal)
// ============================================================================
/*
 * Helper function that performs ONE RMS calculation on a set of samples.
 * Returns the RMS voltage value in millivolts.
 */
float calculateSingleRMS(uint8_t pin, uint16_t samples, int &min_mV, int &max_mV) {
  float sumSquares = 0.0f;
  float sum_mV = 0.0f;
  min_mV = 4095;
  max_mV = 0;

  // Take samples and accumulate squared values
  for (uint16_t i = 0; i < samples; i++) {
    int raw_mV = analogReadMilliVolts(pin);
    sum_mV += raw_mV;
    sumSquares += (float)raw_mV * (float)raw_mV;

    if (raw_mV < min_mV) min_mV = raw_mV;
    if (raw_mV > max_mV) max_mV = raw_mV;

    delayMicroseconds(50);  // Small delay between samples
  }

  // Calculate RMS using mean-centering: rms = sqrt(E[x^2] - (E[x])^2)
  float mean_mV = sum_mV / (float)samples;
  float meanSquares = (sumSquares / (float)samples) - (mean_mV * mean_mV);
  if (meanSquares < 0.0f) {
    meanSquares = 0.0f;
  }
  return sqrtf(meanSquares);
}

// ============================================================================
// HELPER FUNCTION - DC Current Measurement (ACS712T)
// ============================================================================
/*
 * Samples an ADC pin multiple times and calculates DC current.
 * Uses simple averaging of multiple samples for DC measurement.
 * ACS712T can also measure DC - just subtract offset and convert.
 * 
 * PARAMETERS:
 *   pin: ADC pin to sample
 *   samples: Number of samples to average (default 200)
 *   
 * RETURNS:
 *   Current in Amperes (DC)
 *   
 * TIMING:
 *   ~200 samples = ~20-30ms total
 */
float readDC_and_convertToCurrent(uint8_t pin, uint16_t samples = ADC_SAMPLES) {
  float sum_mV = 0.0f;
  int min_mV = 4095;
  int max_mV = 0;

  // Take samples and accumulate values
  for (uint16_t i = 0; i < samples; i++) {
    int raw_mV = analogReadMilliVolts(pin);
    sum_mV += raw_mV;

    if (raw_mV < min_mV) min_mV = raw_mV;
    if (raw_mV > max_mV) max_mV = raw_mV;

    delayMicroseconds(50);  // Small delay between samples
  }

  // Calculate average DC voltage
  float mean_mV = sum_mV / (float)samples;
  float voltage_diff_mV = mean_mV - (ACS712_DC_OFFSET_V * 1000.0f);  // Subtract offset

  // Store debug info for CURR1
  if (pin == CURRENT_SENSOR1) {
    debug_raw_adc_avg = mean_mV;
    debug_rms_curr1 = voltage_diff_mV;  // Voltage difference from offset
    debug_min_mV = (float)min_mV;
    debug_max_mV = (float)max_mV;
  }

  // Convert voltage difference to current using ACS712T sensitivity
  // current = voltage_diff / sensitivity
  float current_A = (voltage_diff_mV / 1000.0f) / ACS712_SENSITIVITY_VpA;

  // Apply per-channel gain trim for calibration matching between channels.
  if (pin == CURRENT_SENSOR1) current_A *= CURRENT_GAIN_CH1;
  else if (pin == CURRENT_SENSOR2) current_A *= CURRENT_GAIN_CH2;
  else if (pin == CURRENT_SENSOR3) current_A *= CURRENT_GAIN_CH3;
  else if (pin == CURRENT_SENSOR4) current_A *= CURRENT_GAIN_CH4;

  // Clamp to valid range (ACS712T rated 0-5A)
  if (current_A < 0.0f) current_A = 0.0f;
  if (current_A > ACS712_MAX_CURRENT_A) current_A = ACS712_MAX_CURRENT_A;

  return current_A;
}

// ============================================================================
// HELPER FUNCTION - AC Current Measurement using Multiple RMS (ACS712T)
// ============================================================================
/*
 * Samples an ADC pin multiple times and calculates RMS AC current.
 * Uses MULTIPLE RMS calculations and averages them for better noise rejection.
 * ACS712T outputs a sinusoidal signal centered on DC offset (1.55V measured with divisor).
 * 
 * PARAMETERS:
 *   pin: ADC pin to sample
 *   samples: Number of samples per RMS calculation (default 200)
 *   iterations: Number of RMS calculations to average (default 5)
 *   
 * RETURNS:
 *   Current in Amperes (RMS - effective AC current)
 *   
 * TIMING:
 *   ~200 samples × 5 iterations = ~100-150ms total (excellent noise reduction)
 */
float readRMS_and_convertToCurrent(uint8_t pin, uint16_t samples = ADC_SAMPLES, uint8_t iterations = RMS_ITERATIONS) {
  float rms_current_sum = 0.0f;
  int min_mV_all = 4095;
  int max_mV_all = 0;

  // Take multiple RMS readings and average them
  for (uint8_t iter = 0; iter < iterations; iter++) {
    int min_mV, max_mV;
    float rms_voltage_mV = calculateSingleRMS(pin, samples, min_mV, max_mV);
    
    // Track overall min/max
    if (min_mV < min_mV_all) min_mV_all = min_mV;
    if (max_mV > max_mV_all) max_mV_all = max_mV;

    // Convert this RMS voltage to current
    float current_A = (rms_voltage_mV / 1000.0f) / ACS712_SENSITIVITY_VpA;

    // Apply per-channel gain trim for calibration matching between channels.
    if (pin == CURRENT_SENSOR1) current_A *= CURRENT_GAIN_CH1;
    else if (pin == CURRENT_SENSOR2) current_A *= CURRENT_GAIN_CH2;
    else if (pin == CURRENT_SENSOR3) current_A *= CURRENT_GAIN_CH3;
    else if (pin == CURRENT_SENSOR4) current_A *= CURRENT_GAIN_CH4;
    
    // Apply software baseline correction (subtract residual ~40mA)
    current_A -= ACS712_BASELINE_CORRECTION_A;
    
    // Clamp to valid range after correction
    if (current_A < 0.0f) current_A = 0.0f;
    if (current_A > ACS712_MAX_CURRENT_A) current_A = ACS712_MAX_CURRENT_A;
    
    rms_current_sum += current_A;
  }

  // Average all RMS current readings
  float current_A_avg = rms_current_sum / (float)iterations;

  // Store debug info for CURR1 (using last RMS calculation values for display)
  if (pin == CURRENT_SENSOR1) {
    debug_raw_adc_avg = ACS712_DC_OFFSET_V * 1000.0f;  // Expected DC offset in mV (1550mV for 5.4V supply)
    debug_rms_curr1 = current_A_avg * ACS712_SENSITIVITY_mVpA;  // Show equivalent RMS voltage
    debug_min_mV = (float)min_mV_all;
    debug_max_mV = (float)max_mV_all;
  }

  return current_A_avg;
}

// ============================================================================
// HELPER FUNCTION - RMS Calculation for AC Voltage Measurement (ZMPT101B)
// ============================================================================
/*
 * Samples ZMPT101B pin multiple times and calculates RMS voltage value.
 * ZMPT101B has fixed 1.65V DC offset with AC signal modulation.
 * 
 * PARAMETERS:
 *   pin: ADC pin to sample
 *   samples: Number of samples to take (default 30)
 *   
 * RETURNS:
 *   Voltage in Volts RMS (0-120V range)
 *   
 * TIMING:
 *   ~30 samples = ~1-2ms (appropriate for 50/60Hz AC sampling)
 */
float readRMS_and_convertToVoltage(uint8_t pin, uint16_t samples = ADC_SAMPLES) {
  float sumSquares = 0.0f;
  float sum_mV = 0.0f;

  // Take samples and accumulate squared values for RMS calculation
  for (uint16_t i = 0; i < samples; i++) {
    int raw_mV = analogReadMilliVolts(pin);

    sum_mV += raw_mV;
    sumSquares += (float)raw_mV * (float)raw_mV;

    delayMicroseconds(100);  // 100µs delay = ~10ms per 100 samples (covers multiple 50/60Hz mains cycles)
  }

  // Calculate RMS using mean-centering: rms = sqrt(E[x^2] - (E[x])^2)
  // This method is robust to DC offset and sensor bias in the data
  float mean_mV = sum_mV / (float)samples;
  float meanSquares = (sumSquares / (float)samples) - (mean_mV * mean_mV);
  if (meanSquares < 0.0f) {
    meanSquares = 0.0f;  // Clamp to zero if numerical error causes negative variance
  }
  float rms_voltage_mV = sqrtf(meanSquares);

  // Apply systematic offset correction: ADC reads ~7mV higher than actual sensor output
  // Root cause: ESP32 ADC and signal conditioning have systematic bias
  // Calibration data:
  //   - Measured mains voltage (multimeter): 119.8V RMS
  //   - ADC pin voltage (multimeter): 102mV RMS
  //   - Calculated by code (before correction): 109mV RMS
  //   - Correction value: -7mV to align with true measurement
  // Without this correction: voltage would read 127V (±5.2% error)
  float rms_voltage_corrected_mV = rms_voltage_mV - ZMPT_OFFSET_mV;
  if (rms_voltage_corrected_mV < 0.0f) {
    rms_voltage_corrected_mV = 0.0f;  // Clamp to zero if correction overshoots
  }

  // Convert RMS voltage (in millivolts) to actual mains voltage using calibration ratio
  // VOLTAGE_RATIO = 1175 V/V means: 1mV at ADC input × 1175 = 1.175V at mains output
  float voltage_V = (rms_voltage_corrected_mV / 1000.0f) * VOLTAGE_RATIO;

  // Optional final site calibration factor (default keeps current 122Vrms behavior)
  voltage_V *= VOLTAGE_CAL_FACTOR;

  return voltage_V;
}

// ============================================================================
// SCALING FUNCTIONS - Convert measurements to 8-bit packed format
// ============================================================================
/*
 * Scales high-precision float measurements to 8-bit (0-255) for packet transmission.
 * This allows full measurements to fit in a 10-byte response packet.
 *
 * Resolution Trade-off:
 *   - Voltage: 0-255 represents 0-130V RMS = 0.51 V/step resolution
 *   - Current: 0-255 represents 0-2550 mA = 10 mA/step resolution
 *
 * The coordinator receives the scaled value and can reconstruct original range.
 * Example: scaled_current = 128 (half scale) = 1275 mA ≈ 1.28A
 */

uint8_t scale_voltage(float voltage_V) {
  // Map 0-130V RMS to 0-255 (resolution: 0.51 V/step)
  if (voltage_V < 0) return 0;
  if (voltage_V > VOLTAGE_SCALE_MAX_V) return 255;
  return (uint8_t)((voltage_V / VOLTAGE_SCALE_MAX_V) * 255.0);
}

uint8_t scale_current(float current_mA) {
  // Map 0-2550 mA to 0-255 (resolution: 10 mA/step)
  if (current_mA < 0) return 0;
  if (current_mA > 2550.0) return 255;
  return (uint8_t)((current_mA / 2550.0) * 255.0);
}

void updateVoltageTxAverage(float voltageSample_V) {
  unsigned long nowMs = millis();

  if (voltageAvgWindowStartMs == 0) {
    voltageAvgWindowStartMs = nowMs;
    AC_voltage_tx_V = voltageSample_V;
  }

  voltageAvgAccumV += voltageSample_V;
  voltageAvgSampleCount++;

  if (nowMs - voltageAvgWindowStartMs >= VOLTAGE_TX_AVG_WINDOW_MS) {
    if (voltageAvgSampleCount > 0) {
      AC_voltage_tx_V = voltageAvgAccumV / voltageAvgSampleCount;
    }
    voltageAvgAccumV = 0.0;
    voltageAvgSampleCount = 0;
    voltageAvgWindowStartMs = nowMs;
  }
}

// ============================================================================
// CHECK FOR LORA PACKETS - Helper Function
// ============================================================================
/*
 * This function checks for incoming LoRa packets and processes them if valid.
 * It's called MULTIPLE TIMES per loop cycle to avoid missing packets while
 * performing sensor readings (~150ms total blocking time for 5 sensors).
 * 
 * STRATEGY: Call between sensor readings to create multiple "RX windows"
 *   - Before sensor readings
 *   - After sensor 1 (30ms gap)
 *   - After sensor 2 (30ms gap)
 *   - After sensor 3 (30ms gap)
 *   - After sensor 4 (30ms gap)
 *   - After voltage sensor (30ms gap)
 * 
 * This reduces max packet loss window from 150ms to ~30ms per window!
 */
void checkLoRaPackets() {
  // Update watchdog timer (activity = checking for packets, even if none arrive)
  lastLoRaActivity = millis();
  
  int packetSize = LoRa.parsePacket();
  
  if (packetSize > 0) {
    // Recibiendo paquete: morado
    neopixel.setPixelColor(0, COLOR_PURPLE);
    neopixel.show();

    // CRITICAL: Don't print here! Wait until we know if packet is for us.
    // Printing at this stage causes ~50ms delay and makes us miss our own packets.

    if (packetSize != 4) {
      // Invalid size - discard and flush buffer QUICKLY
      while (LoRa.available()) {
        LoRa.read(); // Flush without printing to save time
      }
      LoRa.receive(); // Return to RX immediately
      neopixel.setPixelColor(0, COLOR_BLUE);
      neopixel.show();
    } else {
      // Parse the 4-byte request packet
      uint8_t net     = LoRa.read();    // Byte 0: Network ID
      uint8_t type    = LoRa.read();    // Byte 1: Message type
      uint8_t tgtId   = LoRa.read();    // Byte 2: Target device ID
      uint8_t req     = LoRa.read();    // Byte 3: Request code

      // ====================================================================
      // VALIDATE REQUEST (Fast Rejection Strategy)
      // ====================================================================
      // Quick validation: check target ID first, print only if it's for us.
      // This minimizes processing time for packets destined to other nodes.
      // 
      // In a 9-node network, each device receives ~9x more packets than needed.
      // Fast rejection (2-5ms) vs slow rejection with prints (50-100ms) is critical!
      
      if (net == NET_ID && type == MSG_REQ && tgtId == TX_ID && req == REQ_READ_DATA) {
        // ✓ THIS PACKET IS FOR US! Now we can afford to print debug info.
        Serial.printf("📡 Packet for me! RSSI: %d dBm [%02X %02X %02X %02X]\n", 
                      LoRa.packetRssi(), net, type, tgtId, req);
        
        // Scale measurements to 8-bit format for transmission
        uint8_t ac_v_scaled = scale_voltage(AC_voltage_tx_V);
        uint8_t curr1_scaled = scale_current(current_sensor1_A * 1000.0);  // Convert A to mA
        uint8_t curr2_scaled = scale_current(current_sensor2_A * 1000.0);
        uint8_t curr3_scaled = scale_current(current_sensor3_A * 1000.0);
        uint8_t curr4_scaled = scale_current(current_sensor4_A * 1000.0);

        // Espera aleatoria
        delay(random(10, 80));

        // Enviando: cyan
        neopixel.setPixelColor(0, COLOR_CYAN);
        neopixel.show();

        // Build and send response (10-byte packet with scaled values)
        LoRa.beginPacket();
        LoRa.write(NET_ID);                           // Echo network ID
        LoRa.write(MSG_RESP);                         // Message type: Response (0x90)
        LoRa.write(TX_ID);                            // Our device ID
        LoRa.write((uint8_t)(seq & 0xFF));            // Sequence number (low byte)
        LoRa.write((uint8_t)((seq >> 8) & 0xFF));     // Sequence number (high byte)
        LoRa.write(ac_v_scaled);                      // Scaled AC voltage (0-255 = 0-130V RMS)
        LoRa.write(curr1_scaled);                     // Scaled current 1 (0-255 = 0-2550 mA)
        LoRa.write(curr2_scaled);                     // Scaled current 2 (0-255 = 0-2550 mA)
        LoRa.write(curr3_scaled);                     // Scaled current 3 (0-255 = 0-2550 mA)
        LoRa.write(curr4_scaled);                     // Scaled current 4 (0-255 = 0-2550 mA)
        LoRa.endPacket();
        LoRa.receive(); // Ensure radio returns to RX mode

        // Incrementar secuencia
        seq++;

        // Debug output
        Serial.printf("[Device %d] Response sent (Seq=%d): ", TX_ID, seq-1);
        Serial.printf("AC(avg3s)=%.1fV(%d) AC(inst)=%.1fV CURR1=%.2fA(%d) CURR2=%.2fA(%d) CURR3=%.2fA(%d) CURR4=%.2fA(%d)\n",
          AC_voltage_tx_V, ac_v_scaled, AC_voltage_V,
          current_sensor1_A, curr1_scaled,
          current_sensor2_A, curr2_scaled,
          current_sensor3_A, curr3_scaled,
          current_sensor4_A, curr4_scaled);

        // Fin de envío: verde
        neopixel.setPixelColor(0, COLOR_GREEN);
        neopixel.show();
        delay(100);
        // Regresa a standby azul
        neopixel.setPixelColor(0, COLOR_BLUE);
        neopixel.show();
      } else {
        // ✗ NOT for this node (packet for Device 2, 3, 4...9)
        // CRITICAL: Don't print anything here! Return to RX mode IMMEDIATELY.
        // 
        // WHY: In a 9-node network, we receive ~8 packets for other devices
        //      for every 1 packet for us. If we print debug info for rejected
        //      packets, we spend ~50-100ms per rejection and MISS our own packets!
        // 
        // Silent rejection time: ~2-5ms (fast enough to catch our next packet)
        // Verbose rejection time: ~50-100ms (too slow, causes packet loss)
        
        LoRa.receive(); // Return to RX mode immediately (no Serial prints!)
        neopixel.setPixelColor(0, COLOR_BLUE);
        neopixel.show();
      }
    }
  }
}

// ============================================================================
// MAIN LOOP - Continuous Execution
// ============================================================================
/*
 * OPERATION FLOW:
 * 0. LoRa Watchdog: Check for module freeze and auto-reset if needed
 * 1. Check for LoRa packets (before sensor readings)
 * 2. Read sensor 1 → Check LoRa (30ms window)
 * 3. Read sensor 2 → Check LoRa (30ms window)
 * 4. Read sensor 3 → Check LoRa (30ms window)
 * 5. Read sensor 4 → Check LoRa (30ms window)
 * 6. Read voltage → Check LoRa (30ms window)
 * 7. Process/filter data
 * 8. Small delay, then repeat
 *
 * CRITICAL OPTIMIZATIONS:
 *   1. Multiple LoRa checks per loop
 *      - OLD: Sensors readings (~150ms) THEN check LoRa → High packet loss!
 *      - NEW: Check LoRa BETWEEN each sensor → Max 30ms window → Low packet loss!
 *   
 *   2. LoRa Watchdog (Auto-Recovery)
 *      - Monitors LoRa activity every loop cycle
 *      - If no activity for 60s → Automatic hardware reset + re-init
 *      - Prevents permanent freeze conditions
 *      - LED turns RED during reset, returns to BLUE after recovery
 *
 * NOTE: USE_ONLY_SENSOR1 Configuration
 *   - When USE_ONLY_SENSOR1 = true: Only CURRENT_SENSOR1 is active
 *   - CURRENT_SENSOR2/3/4 are set to 0.0A and unused
 *   - This saves processing time and ADC reads
 *   - Debug output shows detailed CURR1 data
 *   - Filters for sensors 2-4 are still allocated but inactive
 *   - To use all 4 sensors: Set USE_ONLY_SENSOR1 = false (line 287)
 */

void loop() {
  // ========================================================================
  // LORA WATCHDOG CHECK
  // ========================================================================
  // Check if LoRa module has been inactive for too long (indicates freeze)
  unsigned long timeSinceLastActivity = millis() - lastLoRaActivity;
  
  if (timeSinceLastActivity > LORA_WATCHDOG_TIMEOUT_MS) {
    // LoRa module appears frozen - attempt reset
    bool resetSuccess = resetLoRaModule();
    
    if (!resetSuccess) {
      // Reset failed - check if we should halt
      if (loraResetCount >= LORA_RESET_RETRY_MAX) {
        Serial.println("\n\033[1;31m╔═══════════════════════════════════════════════╗\033[0m");
        Serial.println("\033[1;31m║   CRITICAL ERROR: LoRa Module Unrecoverable   ║\033[0m");
        Serial.println("\033[1;31m╚═══════════════════════════════════════════════╝\033[0m");
        Serial.printf("Failed to reset LoRa module after %d attempts.\n", LORA_RESET_RETRY_MAX);
        Serial.println("\nPossible causes:");
        Serial.println("  1. LoRa module hardware failure");
        Serial.println("  2. Loose or broken connection (check wiring)");
        Serial.println("  3. SPI bus conflict with other devices");
        Serial.println("  4. Insufficient power supply to LoRa module");
        Serial.println("\nACTION REQUIRED:");
        Serial.println("  - Check physical connections");
        Serial.println("  - Verify 3.3V power supply to LoRa module");
        Serial.println("  - Power cycle the ESP32 (reset button)");
        Serial.println("  - Replace LoRa module if problem persists\n");
        
        // Halt with blinking RED LED
        while (true) {
          neopixel.setPixelColor(0, COLOR_RED);
          neopixel.show();
          digitalWrite(LED_PIN, HIGH);
          delay(500);
          neopixel.setPixelColor(0, COLOR_OFF);
          neopixel.show();
          digitalWrite(LED_PIN, LOW);
          delay(500);
        }
      }
      // Reset failed but retry attempts remain - continue and retry on next timeout
      delay(1000);
      return;
    }
    // Reset successful - continue normal operation
  }

  // Pulso azul en standby para indicar que el loop está activo
  static unsigned long lastPulse = 0;
  static bool pulseState = false;
  unsigned long now = millis();
  if (now - lastPulse > 1000) { // cada 1 segundo
    if (!pulseState) {
      neopixel.setPixelColor(0, COLOR_OFF);
      neopixel.show();
      pulseState = true;
      lastPulse = now;
    } else {
      neopixel.setPixelColor(0, COLOR_BLUE);
      neopixel.show();
      pulseState = false;
      lastPulse = now;
    }
  }

  // ========================================================================
  // CURRENT MEASUREMENT MODE SELECTION - Ternary Operator Example
  // ========================================================================
  // OPERATOR TERNARIO: condición ? valor_si_verdadero : valor_si_falso
  // 
  // Este es un "if/else" comprimido en una sola línea usando el operador ?:
  // 
  // Estructura:
  //   (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(...) : readDC_and_convertToCurrent(...)
  //                 ↑                          ↑                              ↑
  //            CONDICIÓN                 SI VERDADERO                     SI FALSO
  //
  // En este caso:
  //   - Si CURRENT_MEASUREMENT_AC = true  → Usa modo AC (RMS multiple)
  //   - Si CURRENT_MEASUREMENT_AC = false → Usa modo DC (promedio simple)
  //
  // Es equivalente a:
  //   if (CURRENT_MEASUREMENT_AC) {
  //     current_sensor1_A = readRMS_and_convertToCurrent(CURRENT_SENSOR1);
  //   } else {
  //     current_sensor1_A = readDC_and_convertToCurrent(CURRENT_SENSOR1);
  //   }
  //
  // Pero en una sola línea, más compacto. Muy común en C/C++.
  // ========================================================================

  // Check LoRa before readings to catch queued packets
  checkLoRaPackets();

  current_sensor1_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR1) : readDC_and_convertToCurrent(CURRENT_SENSOR1);
  checkLoRaPackets();

  if (USE_ONLY_SENSOR1) {
    current_sensor2_A = 0.0;
    current_sensor3_A = 0.0;
    current_sensor4_A = 0.0;
  } else if (USE_SENSORS_1_AND_3_ONLY) {
    current_sensor2_A = 0.0;
    current_sensor3_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR3) : readDC_and_convertToCurrent(CURRENT_SENSOR3);
    checkLoRaPackets();
    current_sensor4_A = 0.0;
  } else {
    current_sensor2_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR2) : readDC_and_convertToCurrent(CURRENT_SENSOR2);
    checkLoRaPackets();

    current_sensor3_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR3) : readDC_and_convertToCurrent(CURRENT_SENSOR3);
    checkLoRaPackets();

    current_sensor4_A = (CURRENT_MEASUREMENT_AC) ? readRMS_and_convertToCurrent(CURRENT_SENSOR4) : readDC_and_convertToCurrent(CURRENT_SENSOR4);
    checkLoRaPackets();
  }
  AC_voltage_V = readRMS_and_convertToVoltage(AC_POWER_PIN);
  checkLoRaPackets();
  
  // SIMULATION MODE: Fixed 400mA on all channels
  if (SIMULATE_MODE) {
    sim_current1_A = 0.400f;
    sim_current2_A = 0.400f;
    sim_current3_A = 0.400f;
    sim_current4_A = 0.400f;
    sim_voltage_V  = 125.0f;
    current_sensor1_A = sim_current1_A;
    if (!USE_ONLY_SENSOR1 && !USE_SENSORS_1_AND_3_ONLY) {
      current_sensor2_A = sim_current2_A;
      current_sensor3_A = sim_current3_A;
      current_sensor4_A = sim_current4_A;
    } else if (USE_SENSORS_1_AND_3_ONLY) {
      current_sensor2_A = 0.0f;
      current_sensor3_A = sim_current3_A;
      current_sensor4_A = 0.0f;
    }
    AC_voltage_V = sim_voltage_V;
    // Simulated debug values (consistent with 1.55V offset and typical AC readings)
    debug_raw_adc_avg = ACS712_DC_OFFSET_V * 1000.0f;  // 1550mV - current offset
    debug_rms_curr1 = 25.0;  // Simulated RMS voltage swing (typical for ~0.5A AC)
    debug_min_mV = 1450.0;   // Min ADC reading in simulation
    debug_max_mV = 1650.0;   // Max ADC reading in simulation
  }

  // Update 3-second voltage average used for LoRa transmission
  updateVoltageTxAverage(AC_voltage_V);
  
  // Apply moving average filter to smooth readings
  current_sensor1_A = applyMovingAverage(current_sensor1_A, filter_curr1, filter_index1);
  if (!USE_ONLY_SENSOR1 && !USE_SENSORS_1_AND_3_ONLY) {
    current_sensor2_A = applyMovingAverage(current_sensor2_A, filter_curr2, filter_index2);
    current_sensor3_A = applyMovingAverage(current_sensor3_A, filter_curr3, filter_index3);
    current_sensor4_A = applyMovingAverage(current_sensor4_A, filter_curr4, filter_index4);
  } else if (USE_SENSORS_1_AND_3_ONLY) {
    current_sensor3_A = applyMovingAverage(current_sensor3_A, filter_curr3, filter_index3);
    current_sensor2_A = 0.0;
    current_sensor4_A = 0.0;
  }
  
  // Apply noise floor: clamp weak readings to 0A
  if (current_sensor1_A < CURRENT_FLOOR_A) current_sensor1_A = 0.0;
  if (current_sensor2_A < CURRENT_FLOOR_A) current_sensor2_A = 0.0;
  if (current_sensor3_A < CURRENT_FLOOR_A) current_sensor3_A = 0.0;
  if (current_sensor4_A < CURRENT_FLOOR_A) current_sensor4_A = 0.0;

#ifdef FORCE_CURRENT_TEST_A
  // TEST OVERRIDE: replace active-channel readings with fixed value
  current_sensor1_A = FORCE_CURRENT_TEST_A;
  if (USE_SENSORS_1_AND_3_ONLY) current_sensor3_A = FORCE_CURRENT_TEST_A;
#endif

  // Tower control logic:
  // GREEN only when all active current channels are >5mA (sensor dañado, umbral temporal).
  // Otherwise force RED and disable GREEN relay.
  // voltageOk is kept for logging only – not used for state decisions
  const bool voltageOk = AC_voltage_V >= TOWER_VOLTAGE_THRESHOLD_V;
  const bool currentsOk = areActiveCurrentsAboveTowerThreshold();
  const TowerState requestedTowerState = currentsOk ? TOWER_STATE_GREEN : TOWER_STATE_RED;

  if (requestedTowerState != currentTowerState) {
    currentTowerState = requestedTowerState;
    setTowerRelayState(currentTowerState);

    Serial.printf("[TOWER] State=%s | V=%.1fV | I1=%.0fmA I2=%.0fmA I3=%.0fmA I4=%.0fmA\n",
      (currentTowerState == TOWER_STATE_GREEN) ? "GREEN" : "RED",
      AC_voltage_V,
      current_sensor1_A * 1000.0f,
      current_sensor2_A * 1000.0f,
      current_sensor3_A * 1000.0f,
      current_sensor4_A * 1000.0f);
  }
  
  // Update OLED display once per second
  static unsigned long lastOledUpdate = 0;
  if (now - lastOledUpdate >= 1000) {
    updateOLED();
    lastOledUpdate = now;
  }

  // Print readings to serial monitor once per second
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 1000) {
    if (SIMULATE_MODE) {
      Serial.println("*** SIMULATION MODE ACTIVE ***");
    }
    Serial.printf("Current: CURR1=%.2fA CURR2=%.2fA CURR3=%.2fA CURR4=%.2fA | AC Voltage: %.1fV RMS\n",
      current_sensor1_A, current_sensor2_A, current_sensor3_A, current_sensor4_A, AC_voltage_tx_V);

    // Dedicated calibration line for active channels S1 and S3
    float delta_s1_s3_A = current_sensor1_A - current_sensor3_A;
    Serial.printf("[CAL S1-S3] S1=%.0fmA | S3=%.0fmA | DELTA=%.0fmA\n",
      current_sensor1_A * 1000.0f,
      current_sensor3_A * 1000.0f,
      delta_s1_s3_A * 1000.0f);
    
    // Detailed debug output (CURR1 only - others available if USE_ONLY_SENSOR1 is disabled)
    float vpp_mV = debug_max_mV - debug_min_mV;
    const char* mode_str = (CURRENT_MEASUREMENT_AC) ? "AC(RMS×5)" : "DC(Avg)";
    Serial.printf("[DEBUG CURR1] Offset=%.1fmV Diff=%.2fmV Vpp=%.1fmV min=%.0fmV max=%.0fmV -> %.4fA [%s]\n",
      debug_raw_adc_avg, debug_rms_curr1, vpp_mV, debug_min_mV, debug_max_mV, current_sensor1_A, mode_str);
    lastPrint = now;
  }

  // ========================================================================
  // STEP 6: LOOP DELAY
  // ========================================================================
  // Small delay to avoid busy-waiting while polling LoRa module
  delay(50);
}

// ============================================================================
// END OF PROGRAM
// ============================================================================
