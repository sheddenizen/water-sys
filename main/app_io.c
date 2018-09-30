#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/pcnt.h"

#include "app_io.h"


#define DEFAULT_VREF (1100)        // ADC Calibration of last resort

// Pump PWM control on this gpio
#define PUMP_GPIO_PIN (23)

// Flow sensor on this gpio
#define FLOW_GPIO_PIN (16)

// Solenoids 0 - 5 on these GPIO Pin numbers
static uint8_t const solenoid_pin_map[NUM_SOLENOIDS] = {26, 25, 17, 18, 4, 15};

// Bit-mapped solenoid status
static uint32_t solenoid_flags = 0;

#define STATUS_LIGHT_PIN (2)

static esp_adc_cal_characteristics_t *adc_characteristics;

// Various analog input to ADC channel numbers
#define OUTLET_PRESSURE_ADC_CHANNEL (5)
#define TANK_PRESSURE_ADC_CHANNEL (7)
#define PUMP_CURRENT_ADC_CHANNEL (0)
#define SUPPLY_VOLTAGE_ADC_CHANNEL (3)

// List of ADC channels for initialization
static uint8_t const analog_channel_list[] =
{
    OUTLET_PRESSURE_ADC_CHANNEL,
    TANK_PRESSURE_ADC_CHANNEL,
    PUMP_CURRENT_ADC_CHANNEL,
    SUPPLY_VOLTAGE_ADC_CHANNEL
};

#define NUM_ANALOGS (sizeof(analog_channel_list) / sizeof(analog_channel_list[0]))

void appio_init()
{
    // Set all solenoids to output mode and off
    for (int solenoid=0; solenoid < NUM_SOLENOIDS; ++solenoid)
    {
        uint8_t gpio_pin = solenoid_pin_map[solenoid];
        gpio_pad_select_gpio(gpio_pin);
        gpio_set_direction(gpio_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(gpio_pin, 0);
    }

    gpio_pad_select_gpio(STATUS_LIGHT_PIN);
    gpio_set_direction(STATUS_LIGHT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LIGHT_PIN, 0);

    gpio_pad_select_gpio(FLOW_GPIO_PIN);
    gpio_set_direction(FLOW_GPIO_PIN, GPIO_MODE_INPUT);


    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = FLOW_GPIO_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on both edges
        .neg_mode = PCNT_COUNT_INC,
        // Ignore control input
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);


    // Configure ADC1
    adc1_config_width(ADC_WIDTH_BIT_12);

    //Characterize ADC1
    adc_characteristics = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_0, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_characteristics);

    // Set attenuation on the channels we use
    for (int analog = 0; analog < NUM_ANALOGS; ++analog)
    {
        adc1_config_channel_atten(analog_channel_list[analog], ADC_ATTEN_DB_0);
    }

    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }

    // Set up PWM operation for pump - we use the LED control as the motor control module is a bit unecessary for a single-ended control
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 20000,                     // frequency of PWM signal
        .speed_mode = LEDC_HIGH_SPEED_MODE,   // timer mode
        .timer_num = LEDC_TIMER_0             // timer index
    };

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Configure channel
    ledc_channel_config_t ledc_channel = {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = PUMP_GPIO_PIN,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel  = LEDC_TIMER_0
    };

    // Set LED Controller with previously prepared configuration
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    ledc_fade_func_install(0);
}

// Set pump demand 0-1023
void appio_set_pump_drive(uint32_t level)
{
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, level);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

uint32_t appio_get_pump_drive()
{
    return ledc_get_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

// Turn solenoid 0-5 on/off, true for on/open
void appio_set_solenoid(uint32_t solenoid_index, bool open_state)
{
    if (solenoid_index < NUM_SOLENOIDS)
    {
        gpio_set_level(solenoid_pin_map[solenoid_index], open_state);
        solenoid_flags = open_state ? solenoid_flags | ( 1 << solenoid_index ) : solenoid_flags & ~( 1 << solenoid_index );
    }
}

bool appio_get_solenoid(uint32_t solenoid_index)
{
    return (solenoid_flags & (1 << solenoid_index)) != 0;
//    return gpio_get_level(solenoid_pin_map[solenoid_index]);
}

bool appio_any_solenoid_open()
{
    return solenoid_flags != 0;
}

void appio_close_all_solenoids()
{
    for (int n = 0; n < NUM_SOLENOIDS; ++n)
    {
        appio_set_solenoid(n, false);
    }
}

int32_t appio_get_outlet_pressure_mbar()
{

    // 0-150 PSI Sensor, ranging 0.5v - 4.5v
    static const int32_t sensor_fsd_mbar = 10342;
    static const int32_t sensor_fsd_mv = 4000;
    static const int32_t sensor_zero_offset_mv = 500;
    // 33k series to 15k shunt voltage divider
    static const int32_t divider_numerator = 5;
    static const int32_t divider_denominator = (11 + 5);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(OUTLET_PRESSURE_ADC_CHANNEL,ADC_ATTEN_DB_0);
    uint32_t adc_reading = adc1_get_raw(OUTLET_PRESSURE_ADC_CHANNEL);

    //Convert adc_reading to voltage in mV
    int32_t adc_milli_volts = esp_adc_cal_raw_to_voltage(adc_reading, adc_characteristics);

    // Convert to milliBar
    return (adc_milli_volts * divider_denominator - sensor_zero_offset_mv * divider_numerator) *
                sensor_fsd_mbar / divider_denominator / sensor_fsd_mv;

    // Cut crap - look at direct out
//    uint32_t adc_reading = adc1_get_raw(OUTLET_PRESSURE_ADC_CHANNEL);
//    return esp_adc_cal_raw_to_voltage(adc_reading, adc_characteristics);
}


int32_t appio_get_tank_level_percent_x10()
{
    static const int32_t full_value = 2854;
    static const int32_t empty_value = 1722;
    int32_t adc_reading = adc1_get_raw(TANK_PRESSURE_ADC_CHANNEL);

    return (adc_reading - empty_value) * 1000 / (full_value - empty_value);
}

int32_t appio_get_pump_current_milliamps()
{
    static const int32_t shunt_resistance_milliohms = 100;
    static const int32_t offset_ma = 1020;

    uint32_t adc_reading = adc1_get_raw(PUMP_CURRENT_ADC_CHANNEL);

    //Convert adc_reading to voltage in mV
    int32_t adc_milli_volts = esp_adc_cal_raw_to_voltage(adc_reading, adc_characteristics);

    return adc_milli_volts * 1000 / shunt_resistance_milliohms - offset_ma;
}

int32_t appio_get_supply_voltage_millivolts()
{
    // 15k series to 1k shunt voltage divider
    static const int32_t divider_numerator = 1;
    static const int32_t divider_denominator = (15 + 1);

    uint32_t adc_reading = adc1_get_raw(SUPPLY_VOLTAGE_ADC_CHANNEL);

    //Convert adc_reading to voltage in mV
    int32_t adc_milli_volts = esp_adc_cal_raw_to_voltage(adc_reading, adc_characteristics);

    // Scale to resistor combination
    return adc_milli_volts * divider_denominator / divider_numerator;
}


void appio_set_blinkenlight(bool state)
{
    gpio_set_level(STATUS_LIGHT_PIN, state);
}

bool appio_get_flow_sensor_state()
{
    return gpio_get_level(FLOW_GPIO_PIN);
}

uint16_t appio_get_flow_count()
{
    int16_t count;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    return count;
}
