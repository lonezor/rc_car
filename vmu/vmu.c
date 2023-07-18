#include <Wire.h>
#include <DFRobot_ADS1115.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

DFRobot_ADS1115 ads(&Wire);

/** Current I/O state
 *
 * Prefix:
 * - m: momentary button
 * - l: latching button
 * - o: output pin
 */
typedef struct {
    int d10_m_tune_channel_a;
    int d9_m_tune_channel_b;
    int d8_m_tune_channel_c;
    int d7_m_tune_channel_d;
    int d6_m_alarm_config;
    int d5_m_write_eeprom;
    int d4_m_toggle_lcd;
    int d3_m_test_alarm;

    int prev_d10_m_tune_channel_a;
    int prev_d9_m_tune_channel_b;
    int prev_d8_m_tune_channel_c;
    int prev_d7_m_tune_channel_d;
    int prev_d6_m_alarm_config;
    int prev_d5_m_write_eeprom;
    int prev_d4_m_toggle_lcd;
    int prev_d3_m_test_alarm;

    int a0_tuning_potentiometer;

    int intermediate_warning_threshold;
    int intermediate_unsafe_threshold;
    int intermediate_critical_threshold;

    double intermediate_scaling_factor_a;
    double intermediate_scaling_factor_b;
    double intermediate_scaling_factor_c;
    double intermediate_scaling_factor_d;
    
    int eeprom_button_low_counter;

    int a1_reserved;
    int a2_reserved;
    int a3_reserved;
    int a6_reserved;
    int a7_reserved;

    int adc0_battery_a;
    int adc1_battery_b;
    int adc2_battery_c;
    int adc3_battery_d;

    int lcd_backlight;
    int threshold_idx;
} io_state_t;

typedef struct {
    int millivolt_battery_a;
    int millivolt_battery_b;
    int millivolt_battery_c;
    int millivolt_battery_d;
} voltage_measurements_t;

static voltage_measurements_t g_voltage_measurements;
static io_state_t g_io_state;

// Rough scaling factor as a starting value
// This is an initializer value for the EEPROM configuration
// and is adjusted based on reference equipment
#define UNCALIBRATED_SCALING_FACTOR (0.011104477611940299)

#define VOLT_TO_MILLIVOLT_MULTIPLIER (1000)

/* Tuning: Shrink factor / step multiplier combination
 *
 * These values are carefully selected so that the main voltage reference
 * (lab power supply) gets optimal voltage value exactly in the middle of
 * the 0-1024 scale. This gives equal tuning ability in both positive and
 * negative direction.
 *
 * The step multiplier needs a value so that it is easy to control a single
 * millivolt without the need to be too precise in physical rotation. This
 * makes it easy to tune.
 *
 * If for some reason the shrink factor is not enough when changing voltage
 * reference, the tuning window can be adjuster in both directions by setting
 * ADC value 0 or 1023 and flashing EEPROM with the new setting. Then repeating
 * this pattern the window is shifted up or down. 
 */
#define TUNING_SHRINK_FACTOR (0.99795)
#define TUNING_STEP_MULTIPLIER (0.00000004)

typedef enum {
    UI_SCREEN_MAIN,
    UI_SCREEN_TUNING,
    UI_SCREEN_ALARM_THRESHOLDS,
} ui_screen_t;

typedef enum {
    TUNING_CHANNEL_NONE,
    TUNING_CHANNEL_BATTERY_A,
    TUNING_CHANNEL_BATTERY_B,
    TUNING_CHANNEL_BATTERY_C,
    TUNING_CHANNEL_BATTERY_D,
} tuning_channel_t;

#define EEPROM_FINGERPRINT_BYTE_0 0x12
#define EEPROM_FINGERPRINT_BYTE_1 0x23
#define EEPROM_FINGERPRINT_BYTE_2 0x45
#define EEPROM_FINGERPRINT_BYTE_3 0x67

typedef struct {
    uint8_t fingerprint_0;
    uint8_t fingerprint_1;
    uint8_t fingerprint_2;
    uint8_t fingerprint_3;

    double scaling_factor_battery_a;
    double scaling_factor_battery_b;
    double scaling_factor_battery_c;
    double scaling_factor_battery_d;

    int millivolt_threshold_warning;
    int millivolt_threshold_unsafe;
    int millivolt_threshold_critical;

    int recharge_counter;
} eeprom_entry_t;

static eeprom_entry_t g_e;

uint16_t g_iteration_counter;

typedef enum {
  battery_status_normal,
  battery_status_warning,
  battery_status_unsafe,
  battery_status_critical,
} battery_status_t;

int g_battery_status = (int)battery_status_normal;


/* After each idle period, an activation occurs:
 *
 * ..........***..........***..........
 *
 * The idle period is based on the iteration counter
 * so the active period consumes 'idle' duration.
 * 
 * To allow other tasks in the program the non-blocking
 * piezo_tick() is used.
 */
#define PIEZO_IDLE_TICKS_WARNING (64)
#define PIEZO_IDLE_TICKS_ALARM (16)
#define PIEZO_IDLE_TICKS_CRITICAL (4)
#define PIEZO_ACTIVE_TICKS (1)

#define PIEZO_PIN (2)

static int g_piezo_active_ticks = 0;

/**
* https://www.kendrickastro.com/lvc.html
* 
* The truth is that any lead acid battery, be it a Gel Cell, AGM or flooded batteries such as DCBs, should be cut-off at 11.6 volts.
* Not doing so increases the risk of damage to the battery and if taken down deep enough into the batteries charge, will ruin the battery.
* Having said that, the absolute lowest level a lead acid battery can be discharged to, UNDER LOAD, is 10.8 volts but this is not recommended 
*
* Anyone with a battery that appears to be cutting off too early may be damaged through too deep discharge (even once), damaged from poor
* electrolyte maintenance or is old and no longer serviceable. A battery that is discharged to 10.8 volts regularly and then recharged can
* expect to get 354 charging cycles out of the battery. A battery that is discharged to 11.6 volts regularly and then recharged can expect to
* get 900 to 1000 charges out of the battery. A properly maintained battery will last many years.
*/

#define A0_POT_RANGE_BEGIN (0)
#define A0_POT_RANGE_END (1023)
#define BATTERY_THRESHOLD_WARNING_RANGE_BEGIN (11600)
#define BATTERY_THRESHOLD_WARNING_RANGE_END (12600)
#define BATTERY_THRESHOLD_UNSAFE_RANGE_BEGIN (10800) 
#define BATTERY_THRESHOLD_UNSAFE_RANGE_END (11599)
#define BATTERY_THRESHOLD_CRITICAL_RANGE_START (10000)
#define BATTERY_THRESHOLD_CRITICAL_RANGE_END (10799)

static double g_scaling_factor = 0;
static bool g_initial_eeprom_check = false;
static ui_screen_t g_ui_screen = UI_SCREEN_MAIN;
static tuning_channel_t g_tuning_channel = TUNING_CHANNEL_NONE;

void piezo_tick(battery_status_t status)
{
    switch (status) {
        case battery_status_warning:
            if (g_iteration_counter % PIEZO_IDLE_TICKS_WARNING == 0) {
                g_piezo_active_ticks = PIEZO_ACTIVE_TICKS;
            }
            break;
        case battery_status_unsafe:
            if (g_iteration_counter % PIEZO_IDLE_TICKS_ALARM == 0) {
                g_piezo_active_ticks = PIEZO_ACTIVE_TICKS;
            }
            break;
        case battery_status_critical:
            if (g_iteration_counter % PIEZO_IDLE_TICKS_CRITICAL == 0) {
                g_piezo_active_ticks = PIEZO_ACTIVE_TICKS;
            }
            break;
        default:
            break;
    }

    if (g_piezo_active_ticks > 0) {
        digitalWrite(PIEZO_PIN, HIGH);
        g_piezo_active_ticks--;
    } else {
        digitalWrite(PIEZO_PIN, LOW);
    }
}

void setup(void) 
{
    memset(&g_io_state, 0, sizeof(g_io_state));
    memset(&g_voltage_measurements, 0, sizeof(g_voltage_measurements));

    g_io_state.lcd_backlight = 1;
    g_io_state.threshold_idx = 0;

    lcd.init();
    lcd.init();
    lcd.backlight();
    lcd.setCursor(3,0);

    Serial.begin(115200);
    ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS1);   // 0x48
    ads.setGain(eGAIN_TWOTHIRDS);   // 2/3x gain
    ads.setMode(eMODE_SINGLE);       // single-shot mode
    ads.setRate(eRATE_128);          // 128SPS (default)
    ads.setOSMode(eOSMODE_SINGLE);   // Set to start a single-conversion
    ads.init();

    pinMode(2, OUTPUT);
    pinMode(3, INPUT_PULLUP);
    pinMode(4, INPUT_PULLUP);
    pinMode(5, INPUT_PULLUP);
    pinMode(6, INPUT_PULLUP);
    pinMode(7, INPUT_PULLUP);
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    pinMode(10, INPUT_PULLUP);

    g_iteration_counter = 0;
}

static void read_eeprom()
{
  Serial.println("read_eeprom");

  memset(&g_e, 0, sizeof(g_e));

  int address = 0;

  g_e.fingerprint_0 = EEPROM.read(address++);
  g_e.fingerprint_1 = EEPROM.read(address++);
  g_e.fingerprint_2 = EEPROM.read(address++);
  g_e.fingerprint_3 = EEPROM.read(address++);

  uint8_t* p = (uint8_t*)&g_e.scaling_factor_battery_a;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_a); i++) {
    p[i] = EEPROM.read(address++);
  }

  p = (uint8_t*)&g_e.scaling_factor_battery_b;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_b); i++) {
    p[i] = EEPROM.read(address++);
  }

  p = (uint8_t*)&g_e.scaling_factor_battery_c;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_c); i++) {
    p[i] = EEPROM.read(address++);
  }

  p = (uint8_t*)&g_e.scaling_factor_battery_d;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_d); i++) {
    p[i] = EEPROM.read(address++);
  }

  p = (uint8_t*)&g_e.millivolt_threshold_warning;
  for(int i = 0; i < sizeof(g_e.millivolt_threshold_warning); i++) {
    p[i] = EEPROM.read(address++);
  }

  p = (uint8_t*)&g_e.millivolt_threshold_unsafe;
  for(int i = 0; i < sizeof(g_e.millivolt_threshold_unsafe); i++) {
    p[i] = EEPROM.read(address++);
  }

  p = (uint8_t*)&g_e.millivolt_threshold_critical;
  for(int i = 0; i < sizeof(g_e.millivolt_threshold_critical); i++) {
    p[i] = EEPROM.read(address++);
  }

  p = (uint8_t*)&g_e.recharge_counter;
  for(int i = 0; i < sizeof(g_e.recharge_counter); i++) {
    p[i] = EEPROM.read(address++);
  }
}

static void write_eeprom(eeprom_entry_t* entry)
{
  Serial.println("write_eeprom");

  int address = 0;

  EEPROM.write(address++, EEPROM_FINGERPRINT_BYTE_0);
  EEPROM.write(address++, EEPROM_FINGERPRINT_BYTE_1);
  EEPROM.write(address++, EEPROM_FINGERPRINT_BYTE_2);
  EEPROM.write(address++, EEPROM_FINGERPRINT_BYTE_3);

  uint8_t* p = (uint8_t*)&g_e.scaling_factor_battery_a;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_a); i++) {
    EEPROM.write(address++, p[i]);
  }

  p = (uint8_t*)&g_e.scaling_factor_battery_b;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_b); i++) {
    EEPROM.write(address++, p[i]);
  }

  p = (uint8_t*)&g_e.scaling_factor_battery_c;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_c); i++) {
    EEPROM.write(address++, p[i]);
  }

  p = (uint8_t*)&g_e.scaling_factor_battery_d;
  for(int i = 0; i < sizeof(g_e.scaling_factor_battery_d); i++) {
    EEPROM.write(address++, p[i]);
  }

  p = (uint8_t*)&g_e.millivolt_threshold_warning;
  for(int i = 0; i < sizeof(g_e.millivolt_threshold_warning); i++) {
    EEPROM.write(address++, p[i]);
  }

  p = (uint8_t*)&g_e.millivolt_threshold_unsafe;
  for(int i = 0; i < sizeof(g_e.millivolt_threshold_unsafe); i++) {
    EEPROM.write(address++, p[i]);
  }

  p = (uint8_t*)&g_e.millivolt_threshold_critical;
  for(int i = 0; i < sizeof(g_e.millivolt_threshold_critical); i++) {
    EEPROM.write(address++, p[i]);
  }

  p = (uint8_t*)&g_e.recharge_counter;
  for(int i = 0; i < sizeof(g_e.recharge_counter); i++) {
    EEPROM.write(address++, p[i]);
  }
}

void transform_battery_adc_values_to_voltage_measurements()
{

    //////////////////////////////////////////////////////////////////
    //////////////////// RETRIEVE SCALING FACTORS ////////////////////
    //////////////////////////////////////////////////////////////////
  
    if (!g_initial_eeprom_check) {
        read_eeprom();
        bool match = true;
        match &= g_e.fingerprint_0 == EEPROM_FINGERPRINT_BYTE_0;
        match &= g_e.fingerprint_1 == EEPROM_FINGERPRINT_BYTE_1;
        match &= g_e.fingerprint_2 == EEPROM_FINGERPRINT_BYTE_2;
        match &= g_e.fingerprint_3 == EEPROM_FINGERPRINT_BYTE_3;

        // First EEPROM flash case: no need for UI screen since
        // it is only happens once
        if (!match) {
            memset(&g_e, 0, sizeof(g_e));
      
            g_e.fingerprint_0 = EEPROM_FINGERPRINT_BYTE_0;
            g_e.fingerprint_1 = EEPROM_FINGERPRINT_BYTE_1;
            g_e.fingerprint_2 = EEPROM_FINGERPRINT_BYTE_2;
            g_e.fingerprint_3 = EEPROM_FINGERPRINT_BYTE_3;

            g_e.scaling_factor_battery_a = UNCALIBRATED_SCALING_FACTOR;
            g_e.scaling_factor_battery_b = UNCALIBRATED_SCALING_FACTOR;
            g_e.scaling_factor_battery_c = UNCALIBRATED_SCALING_FACTOR;
            g_e.scaling_factor_battery_d = UNCALIBRATED_SCALING_FACTOR;

            write_eeprom(&g_e);
        }

        g_initial_eeprom_check = true;
    }

    //////////////////////////////////////////////////////////////////
    //////////////////// TRANSFORM SCALING FACTORS ///////////////////
    //////////////////////////////////////////////////////////////////

    g_io_state.intermediate_scaling_factor_a = g_e.scaling_factor_battery_a;
    g_io_state.intermediate_scaling_factor_b = g_e.scaling_factor_battery_b;
    g_io_state.intermediate_scaling_factor_c = g_e.scaling_factor_battery_c;
    g_io_state.intermediate_scaling_factor_d = g_e.scaling_factor_battery_d;

    // Tuning flow
    // Do not update 'g_e' unless flash button is pressed. This
    // allows the tuning value to be discarded when exiting the mode
    switch (g_tuning_channel) {
        case TUNING_CHANNEL_BATTERY_A:
            g_io_state.intermediate_scaling_factor_a *= TUNING_SHRINK_FACTOR;
            g_io_state.intermediate_scaling_factor_a += (g_io_state.a0_tuning_potentiometer * TUNING_STEP_MULTIPLIER);
            break;
        case TUNING_CHANNEL_BATTERY_B:
            g_io_state.intermediate_scaling_factor_b *= TUNING_SHRINK_FACTOR;
            g_io_state.intermediate_scaling_factor_b += (g_io_state.a0_tuning_potentiometer * TUNING_STEP_MULTIPLIER);
            break;
        case TUNING_CHANNEL_BATTERY_C:
            g_io_state.intermediate_scaling_factor_c *= TUNING_SHRINK_FACTOR;
            g_io_state.intermediate_scaling_factor_c += (g_io_state.a0_tuning_potentiometer * TUNING_STEP_MULTIPLIER);
            break;
        case TUNING_CHANNEL_BATTERY_D:
            g_io_state.intermediate_scaling_factor_d *= TUNING_SHRINK_FACTOR;
            g_io_state.intermediate_scaling_factor_d += (g_io_state.a0_tuning_potentiometer * TUNING_STEP_MULTIPLIER);
            break;
    }

    //////////////////////////////////////////////////////////////////////
    //////////////////// PREPARE VOLTAGE MEASUREMENTS ////////////////////
    //////////////////////////////////////////////////////////////////////

    double v = (double)g_io_state.adc0_battery_a;
    v *= g_io_state.intermediate_scaling_factor_a;
    v *= VOLT_TO_MILLIVOLT_MULTIPLIER;
    g_voltage_measurements.millivolt_battery_a = (int)v;

    v = (double)g_io_state.adc1_battery_b;
    v *= g_io_state.intermediate_scaling_factor_b;
    v *= VOLT_TO_MILLIVOLT_MULTIPLIER;
    g_voltage_measurements.millivolt_battery_b = (int)v;

    v = (double)g_io_state.adc2_battery_c;
    v *= g_io_state.intermediate_scaling_factor_c;
    v *= VOLT_TO_MILLIVOLT_MULTIPLIER;
    g_voltage_measurements.millivolt_battery_c = (int)v;

    v = (double)g_io_state.adc3_battery_d;
    v *= g_io_state.intermediate_scaling_factor_d;
    v *= VOLT_TO_MILLIVOLT_MULTIPLIER;
    g_voltage_measurements.millivolt_battery_d = (int)v;
}

void lcd_update()
{
    char msg[512];

    switch (g_ui_screen) {
        case UI_SCREEN_MAIN: {
            int battery_a_volts = g_voltage_measurements.millivolt_battery_a / 1000;
            int battery_b_volts = g_voltage_measurements.millivolt_battery_b / 1000;
            int battery_c_volts = g_voltage_measurements.millivolt_battery_c / 1000;
            int battery_d_volts = g_voltage_measurements.millivolt_battery_d / 1000;
            int battery_a_digits = (g_voltage_measurements.millivolt_battery_a % 1000) / 10;
            int battery_b_digits = (g_voltage_measurements.millivolt_battery_b % 1000) / 10;
            int battery_c_digits = (g_voltage_measurements.millivolt_battery_c % 1000) / 10;
            int battery_d_digits = (g_voltage_measurements.millivolt_battery_d % 1000) / 10;

            char piezo_mode[4];
            snprintf(piezo_mode, sizeof(piezo_mode), "  ");
            if (g_battery_status > 0) {
                snprintf(piezo_mode, sizeof(piezo_mode), "P%d", g_battery_status);
            }

            snprintf(msg, sizeof(msg), "A: %02d.%02d          %s", battery_a_volts, battery_a_digits, piezo_mode);
            lcd.setCursor(0,0);
            lcd.print(msg);

            snprintf(msg, sizeof(msg), "B: %02d.%02d", battery_b_volts, battery_b_digits);
            lcd.setCursor(0,1);
            lcd.print(msg);

            snprintf(msg, sizeof(msg), "C: %02d.%02d", battery_c_volts, battery_c_digits);
            lcd.setCursor(0,2);
            lcd.print(msg);

            snprintf(msg, sizeof(msg), "D: %02d.%02d        %04d", battery_d_volts, battery_d_digits, g_e.recharge_counter);
            lcd.setCursor(0,3);
            lcd.print(msg);
            break;
        }
        case UI_SCREEN_TUNING: {
            int volts = 0;
            int digits = 0;
            char c = 0;

            switch (g_tuning_channel) {
                case TUNING_CHANNEL_BATTERY_A:
                    c = 'A';
                    volts = g_voltage_measurements.millivolt_battery_a / 1000;
                    digits = g_voltage_measurements.millivolt_battery_a % 1000;
                    break;
                case TUNING_CHANNEL_BATTERY_B:
                    c = 'B';
                    volts = g_voltage_measurements.millivolt_battery_b / 1000;
                    digits = g_voltage_measurements.millivolt_battery_b % 1000;
                    break;
                case TUNING_CHANNEL_BATTERY_C:
                    c = 'C';
                    volts = g_voltage_measurements.millivolt_battery_c / 1000;
                    digits = g_voltage_measurements.millivolt_battery_c % 1000;
                    break;
                case TUNING_CHANNEL_BATTERY_D:
                    c = 'D';
                    volts = g_voltage_measurements.millivolt_battery_d / 1000;
                    digits = g_voltage_measurements.millivolt_battery_d % 1000;
                    break;
              }

              snprintf(msg, sizeof(msg), "%c: %02d.%03d", 
                        c,
                        volts,
                        digits);

              lcd.setCursor(0,0);
              lcd.print(msg);
            break;
        }
        case UI_SCREEN_ALARM_THRESHOLDS: {
            char c;
            int volts = 0;
            int digits = 0;

            snprintf(msg, sizeof(msg), "Voltage Thresholds:");
            lcd.setCursor(0,0);
            lcd.print(msg);

            if (g_io_state.threshold_idx == 1) {
                volts = g_io_state.intermediate_warning_threshold / 1000;
                digits = (g_io_state.intermediate_warning_threshold % 1000) / 10;
                c = '>';
            } else {
                volts = g_e.millivolt_threshold_warning / 1000;
                digits = (g_e.millivolt_threshold_warning % 1000) / 10;
                c = '-';
            }
            snprintf(msg, sizeof(msg), " %c Warning:  %02d.%02d", c, volts, digits);
            lcd.setCursor(0,1);
            lcd.print(msg);

            if (g_io_state.threshold_idx == 2) {
                volts = g_io_state.intermediate_unsafe_threshold / 1000;
                digits = (g_io_state.intermediate_unsafe_threshold % 1000) / 10;
                c = '>';
            } else {
              volts = g_e.millivolt_threshold_unsafe / 1000;
              digits = (g_e.millivolt_threshold_unsafe % 1000) / 10;
              c = '-';
            }
            snprintf(msg, sizeof(msg), " %c Unsafe:   %02d.%02d", c, volts, digits);
            lcd.setCursor(0,2);
            lcd.print(msg);

            if (g_io_state.threshold_idx == 3) {
                volts = g_io_state.intermediate_critical_threshold / 1000;
                digits = (g_io_state.intermediate_critical_threshold % 1000) / 10;
                c = '>';
            } else {
              volts = g_e.millivolt_threshold_critical / 1000;
              digits = (g_e.millivolt_threshold_critical % 1000) / 10;
              c = '-';
            }
            snprintf(msg, sizeof(msg), " %c Critical: %02d.%02d", c, volts, digits);
            lcd.setCursor(0,3);
            lcd.print(msg);
            break;
        }
    }
}

void transform_tuning_pot_to_thresholds()
{
    if (g_io_state.threshold_idx == 1) {
        g_io_state.intermediate_warning_threshold = map(g_io_state.a0_tuning_potentiometer, A0_POT_RANGE_BEGIN, A0_POT_RANGE_END, BATTERY_THRESHOLD_WARNING_RANGE_BEGIN, BATTERY_THRESHOLD_WARNING_RANGE_END);
    } else if (g_io_state.threshold_idx == 2) {
        g_io_state.intermediate_unsafe_threshold = map(g_io_state.a0_tuning_potentiometer, A0_POT_RANGE_BEGIN, A0_POT_RANGE_END, BATTERY_THRESHOLD_UNSAFE_RANGE_BEGIN, BATTERY_THRESHOLD_UNSAFE_RANGE_END);
    } else if (g_io_state.threshold_idx == 3) {
        g_io_state.intermediate_critical_threshold = map(g_io_state.a0_tuning_potentiometer, A0_POT_RANGE_BEGIN, A0_POT_RANGE_END, BATTERY_THRESHOLD_CRITICAL_RANGE_START, BATTERY_THRESHOLD_CRITICAL_RANGE_END);
    }
}

void loop(void) 
{
   
    g_io_state.a0_tuning_potentiometer = analogRead(A0);
    g_io_state.d10_m_tune_channel_a = digitalRead(10);
    g_io_state.d9_m_tune_channel_b = digitalRead(9);
    g_io_state.d8_m_tune_channel_c = digitalRead(8);
    g_io_state.d7_m_tune_channel_d = digitalRead(7);
    g_io_state.d6_m_alarm_config = digitalRead(6);
    g_io_state.d5_m_write_eeprom = digitalRead(5);
    g_io_state.d4_m_toggle_lcd = digitalRead(4);
    g_io_state.d3_m_test_alarm  = digitalRead(3);

    char m[512];

    if (ads.checkADS1115()) {
        int16_t adc0, adc1, adc2, adc3;

        if (g_ui_screen == UI_SCREEN_MAIN ||
            (g_ui_screen == UI_SCREEN_ALARM_THRESHOLDS && g_battery_status > 0)) {
          // Improve button read accuracy
          uint16_t selector = g_iteration_counter % 4;
            if (selector == 0) {
              g_io_state.adc0_battery_a = ads.readVoltage(0);
            }

            if (selector == 1) {
                g_io_state.adc1_battery_b = ads.readVoltage(1);
            }

            if (selector == 2) {
                g_io_state.adc2_battery_c = ads.readVoltage(2);
            }

            if (selector == 3) {
                g_io_state.adc3_battery_d = ads.readVoltage(3);
            }
        } else {
          if (g_tuning_channel == TUNING_CHANNEL_BATTERY_A) {
              g_io_state.adc0_battery_a = ads.readVoltage(0);
          } else if (g_tuning_channel == TUNING_CHANNEL_BATTERY_B) {
              g_io_state.adc1_battery_b = ads.readVoltage(1);
          } else if (g_tuning_channel == TUNING_CHANNEL_BATTERY_C) {
              g_io_state.adc2_battery_c = ads.readVoltage(2);
          } else if (g_tuning_channel == TUNING_CHANNEL_BATTERY_D) {
              g_io_state.adc3_battery_d = ads.readVoltage(3);
          }
        }

        transform_battery_adc_values_to_voltage_measurements();
    }

    transform_tuning_pot_to_thresholds();

    if (g_voltage_measurements.millivolt_battery_a < g_e.millivolt_threshold_critical ||
               g_voltage_measurements.millivolt_battery_b < g_e.millivolt_threshold_critical ||
               g_voltage_measurements.millivolt_battery_c < g_e.millivolt_threshold_critical ||
               g_voltage_measurements.millivolt_battery_d < g_e.millivolt_threshold_critical) {
                 g_battery_status = 3;
    } else if (g_voltage_measurements.millivolt_battery_a < g_e.millivolt_threshold_unsafe ||
               g_voltage_measurements.millivolt_battery_b < g_e.millivolt_threshold_unsafe ||
               g_voltage_measurements.millivolt_battery_c < g_e.millivolt_threshold_unsafe ||
               g_voltage_measurements.millivolt_battery_d < g_e.millivolt_threshold_unsafe) {
                 g_battery_status = 2;
    } else if (g_voltage_measurements.millivolt_battery_a < g_e.millivolt_threshold_warning ||
               g_voltage_measurements.millivolt_battery_b < g_e.millivolt_threshold_warning ||
               g_voltage_measurements.millivolt_battery_c < g_e.millivolt_threshold_warning ||
               g_voltage_measurements.millivolt_battery_d < g_e.millivolt_threshold_warning) {
                 g_battery_status = 1;
    } else {
      if (g_io_state.prev_d3_m_test_alarm == HIGH &&
        g_io_state.d3_m_test_alarm == LOW) {
          g_battery_status++;
          if (g_battery_status > (int)battery_status_critical) {
            g_battery_status = (int)battery_status_normal;
          }
      }
    }

    if (g_io_state.lcd_backlight == 0 &&
        g_ui_screen == UI_SCREEN_MAIN &&
        g_io_state.prev_d5_m_write_eeprom == LOW &&
        g_io_state.d5_m_write_eeprom == LOW) {
        if (g_io_state.eeprom_button_low_counter++ > 50) {
            g_io_state.lcd_backlight = !g_io_state.lcd_backlight;
            lcd.display();
            lcd.backlight();

            lcd.clear();


          g_e.recharge_counter = 0;        

          write_eeprom(&g_e);  

          snprintf(m, sizeof(m), "Resetting recharge");
          lcd.setCursor(0,0);
          lcd.print(m);

          snprintf(m, sizeof(m), "      counter");
          lcd.setCursor(0,1);
          lcd.print(m);


          snprintf(m, sizeof(m), " Writing to EEPROM");
          lcd.setCursor(0,3);
          lcd.print(m);

          delay(4000);

          lcd.clear();

          g_io_state.eeprom_button_low_counter = 0;
        }
    } else if (g_ui_screen == UI_SCREEN_MAIN &&
        g_io_state.prev_d5_m_write_eeprom == LOW &&
        g_io_state.d5_m_write_eeprom == LOW) {
        if (g_io_state.eeprom_button_low_counter++ > 50) {
            g_e.recharge_counter++;

            lcd.clear();

          write_eeprom(&g_e);

          snprintf(m, sizeof(m), "  Incrementing");
          lcd.setCursor(0,0);
          lcd.print(m);

          snprintf(m, sizeof(m), " recharge counter");
          lcd.setCursor(0,1);
          lcd.print(m);

          snprintf(m, sizeof(m), " Writing to EEPROM");
          lcd.setCursor(0,3);
          lcd.print(m);


            delay(4000);

            lcd.clear();

            g_io_state.eeprom_button_low_counter = 0;
        }
    } else if (g_ui_screen == UI_SCREEN_TUNING &&
        g_io_state.prev_d5_m_write_eeprom == LOW &&
        g_io_state.d5_m_write_eeprom == LOW) {
        if (g_io_state.eeprom_button_low_counter++ > 1) {
          lcd.clear();

          if (g_tuning_channel == TUNING_CHANNEL_BATTERY_A) {
            g_e.scaling_factor_battery_a =  g_io_state.intermediate_scaling_factor_a;
          } else if (g_tuning_channel == TUNING_CHANNEL_BATTERY_B) {
            g_e.scaling_factor_battery_b =  g_io_state.intermediate_scaling_factor_b;
          } else if (g_tuning_channel == TUNING_CHANNEL_BATTERY_C) {
            g_e.scaling_factor_battery_c =  g_io_state.intermediate_scaling_factor_c;
          } else if (g_tuning_channel == TUNING_CHANNEL_BATTERY_D) {
            g_e.scaling_factor_battery_d =  g_io_state.intermediate_scaling_factor_d;
          }

          write_eeprom(&g_e);
            
          snprintf(m, sizeof(m), "Writing to EEPROM");
          lcd.setCursor(0,0);
          lcd.print(m);

          delay(3000);

          g_ui_screen = UI_SCREEN_MAIN;
          g_tuning_channel = TUNING_CHANNEL_NONE;
          lcd.clear();
           
          g_io_state.eeprom_button_low_counter = 0;
        }
    } else if (g_ui_screen == UI_SCREEN_ALARM_THRESHOLDS &&
        g_io_state.prev_d5_m_write_eeprom == LOW &&
        g_io_state.d5_m_write_eeprom == LOW &&
        g_io_state.threshold_idx != 0) {
        if (g_io_state.eeprom_button_low_counter++ > 1) {

          if (g_io_state.threshold_idx == 1) {
            g_e.millivolt_threshold_warning = g_io_state.intermediate_warning_threshold;
          } else if (g_io_state.threshold_idx == 2) {
            g_e.millivolt_threshold_unsafe = g_io_state.intermediate_unsafe_threshold;
          } else if (g_io_state.threshold_idx == 3) {
            g_e.millivolt_threshold_critical = g_io_state.intermediate_critical_threshold;
          }
 
          write_eeprom(&g_e);

          lcd.clear();
            
          snprintf(m, sizeof(m), "Writing to EEPROM");

          lcd.setCursor(0,0);
          lcd.print(m);

          delay(3000);

          g_ui_screen = UI_SCREEN_MAIN;
          lcd.clear();
           
          g_io_state.eeprom_button_low_counter = 0;
        }
      }

    piezo_tick((battery_status_t)g_battery_status);

    if (g_io_state.prev_d10_m_tune_channel_a == HIGH &&
        g_io_state.d10_m_tune_channel_a == LOW) {

        if (g_ui_screen == UI_SCREEN_MAIN) {
          g_ui_screen = UI_SCREEN_TUNING;
          g_tuning_channel = TUNING_CHANNEL_BATTERY_A;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_TUNING) {
          g_ui_screen = UI_SCREEN_MAIN;
          g_tuning_channel = TUNING_CHANNEL_NONE;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_ALARM_THRESHOLDS) {
          g_ui_screen = UI_SCREEN_MAIN;
          lcd.clear();
        }
    }

    if (g_io_state.prev_d9_m_tune_channel_b == HIGH &&
        g_io_state.d9_m_tune_channel_b == LOW) {

        if (g_ui_screen == UI_SCREEN_MAIN) {
          g_ui_screen = UI_SCREEN_TUNING;
          g_tuning_channel = TUNING_CHANNEL_BATTERY_B;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_TUNING) {
          g_ui_screen = UI_SCREEN_MAIN;
          g_tuning_channel = TUNING_CHANNEL_NONE;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_ALARM_THRESHOLDS) {
          g_ui_screen = UI_SCREEN_MAIN;
          lcd.clear();
        }
    }

    if (g_io_state.prev_d8_m_tune_channel_c == HIGH &&
        g_io_state.d8_m_tune_channel_c == LOW) {

        if (g_ui_screen == UI_SCREEN_MAIN) {
          g_ui_screen = UI_SCREEN_TUNING;
          g_tuning_channel = TUNING_CHANNEL_BATTERY_C;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_TUNING) {
          g_ui_screen = UI_SCREEN_MAIN;
          g_tuning_channel = TUNING_CHANNEL_NONE;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_ALARM_THRESHOLDS) {
          g_ui_screen = UI_SCREEN_MAIN;
          lcd.clear();
        }
    }

    if (g_io_state.prev_d7_m_tune_channel_d == HIGH &&
        g_io_state.d7_m_tune_channel_d == LOW) {

        if (g_ui_screen == UI_SCREEN_MAIN) {
          g_ui_screen = UI_SCREEN_TUNING;
          g_tuning_channel = TUNING_CHANNEL_BATTERY_D;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_TUNING) {
          g_ui_screen = UI_SCREEN_MAIN;
          g_tuning_channel = TUNING_CHANNEL_NONE;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_ALARM_THRESHOLDS) {
          g_ui_screen = UI_SCREEN_MAIN;
          lcd.clear();
        }
    }

    if (g_io_state.prev_d6_m_alarm_config == HIGH &&
        g_io_state.d6_m_alarm_config == LOW) {

        if (g_ui_screen == UI_SCREEN_MAIN) {
          g_ui_screen = UI_SCREEN_ALARM_THRESHOLDS;
          g_io_state.threshold_idx = 0;
          lcd.clear();
        } else if (g_ui_screen == UI_SCREEN_ALARM_THRESHOLDS) {
          g_io_state.threshold_idx++;
          if (g_io_state.threshold_idx > 3) {
            g_ui_screen = UI_SCREEN_MAIN;
            lcd.clear();
          }
        }
    }

    // Restrict LCD updates
    bool update_lcd = false;
    if (g_ui_screen == UI_SCREEN_MAIN) {
      if (g_iteration_counter % 10 == 0) {
        update_lcd = true;
      }
    } else {
      if (g_iteration_counter % 5 == 0) {
        update_lcd = true;
      }
    }

    if (g_io_state.prev_d4_m_toggle_lcd == HIGH &&
        g_io_state.d4_m_toggle_lcd == LOW) {
      update_lcd = false;
          
      g_io_state.lcd_backlight = !g_io_state.lcd_backlight;
    }


    if (update_lcd) {
      lcd_update();
    } else {
      if (g_io_state.lcd_backlight) {
        lcd.display();
        lcd.backlight();
      } else {
        lcd.noDisplay();
        lcd.noBacklight();
      }
    }

    g_io_state.prev_d10_m_tune_channel_a = g_io_state.d10_m_tune_channel_a;
    g_io_state.prev_d9_m_tune_channel_b = g_io_state.d9_m_tune_channel_b;
    g_io_state.prev_d8_m_tune_channel_c = g_io_state.d8_m_tune_channel_c;
    g_io_state.prev_d7_m_tune_channel_d = g_io_state.d7_m_tune_channel_d;
    g_io_state.prev_d6_m_alarm_config = g_io_state.d6_m_alarm_config;
    g_io_state.prev_d5_m_write_eeprom = g_io_state.d5_m_write_eeprom;
    g_io_state.prev_d4_m_toggle_lcd = g_io_state.d4_m_toggle_lcd;
    g_io_state.prev_d3_m_test_alarm = g_io_state.d3_m_test_alarm;

  g_iteration_counter++;
}
