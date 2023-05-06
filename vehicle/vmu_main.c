#include "vehicle.h"
#include "piezo.h"

#define SAMPLE_SIZE (64)
static float a0[SAMPLE_SIZE];
int sample_idx = 0;

static float voltage_a_int = 10;
static float voltage_b_int = 11;
static float voltage_c_int = 12;
static float voltage_d_int = 13;


static float voltage_a_decimals = 4;
static float voltage_b_decimals = 5;
static float voltage_c_decimals = 6;
static float voltage_d_decimals = 7;

void vmu_setup()
{
  //lcd_i2c_init(true);
  //analogReference(EXTERNAL);

//    memset(a0, 0, sizeof(a0));

    pinMode(7, OUTPUT);

}



uint16_t g_iteration_counter = 0;



void vmu_loop()
{
    piezo_tick(battery_status_normal);

    g_iteration_counter++;

    delay(1);


    #if 0
    a0[sample_idx] = (float)analogRead(A0);

    sample_idx++;
    if (sample_idx >= SAMPLE_SIZE) {
        sample_idx = 0;
    }

/** Instead of calculating theoretical value,
 *  use scaling factor against actual value
 *  in lab power supply specifically for the
 *  VMU unit.
 */
#define CALIBRATED_SCALING_FACTOR (0.01508682328907048)
                                   

    float sum = 0;
    int used_samples = 0;
    for(int i=0; i < SAMPLE_SIZE; i++) {
        if (a0[i] > 0) {
            sum += a0[i];
            used_samples++;
        }
    }
    float avg = sum / (float)used_samples;

    float adc_a = avg;
    float adc_a_scaled = adc_a * (float)CALIBRATED_SCALING_FACTOR;

    voltage_a_int = adc_a_scaled;
    voltage_a_decimals = voltage_a_int - (float)((uint32_t)voltage_a_int);
    voltage_a_decimals *= 100;

    char line_01[24];
    snprintf(line_01, sizeof(line_01),
             "Raw: %d %02d %02d  ",
             (int)avg, (int)sample_idx, (int)used_samples);
    lcd_i2c_set_cursor(0,0);
    lcd_i2c_print(line_01);

    snprintf(line_01, sizeof(line_01),
             "A: %02d.%02dV    ",
             (int)voltage_a_int,
             ((int)voltage_a_decimals % 100));
    lcd_i2c_set_cursor(0,1);
    lcd_i2c_print(line_01);


    char line_02[24];
    snprintf(line_02, sizeof(line_02),
             "B: %d.%02dV",
             voltage_b_int, voltage_b_decimals % 100);
    lcd_i2c_set_cursor(0,1);
    lcd_i2c_print(line_02);

    char line_03[24];
    snprintf(line_03, sizeof(line_03),
             "C: %d.%02dV",
             voltage_c_int, voltage_c_decimals % 100);
    lcd_i2c_set_cursor(0,2);
    lcd_i2c_print(line_03);

    char line_04[24];
    snprintf(line_04, sizeof(line_04),
             "D: %d.%02dV",
             voltage_d_int, voltage_d_decimals % 100);
    lcd_i2c_set_cursor(0,3);
    lcd_i2c_print(line_04);

    voltage_a_decimals++;
    voltage_b_decimals++;
    voltage_c_decimals++;
    voltage_d_decimals++;
    #endif

   // // LCD updates only looks okay when at least 500 ms delay is used
   // delay(200);
}