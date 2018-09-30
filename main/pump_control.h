#ifndef PUMP_CONTROL_H
#define PUMP_CONTROL_H

#include "stdint.h"
#include "stdbool.h"

typedef enum
{
    PUMPCTL_IDLE,
    PUMPCTL_STARTING,
    PUMPCTL_STOPPING,
    PUMPCTL_UNSTABLE,
    PUMPCTL_STABLE,
    PUMPCTL_E_STOP
} pumpctl_state_t;


typedef struct pumpctl_data
{
    uint16_t time_ms;
    int16_t pressure_mbar;
    int16_t current_milliamps;
    int16_t drive_mv;
    int16_t power_supply_mv;
    int16_t tank_level_percent_x10;
    int16_t integrator_mbar;
    int16_t setpoint;
    int16_t state;
    int16_t loop_min_us;
    int16_t loop_max_us;

} pumpctl_data_t;

// Pump measurement data handler type - time sensitive, must not block
typedef bool (*pumpctl_status_notify_fn_t)(pumpctl_data_t const *data);

typedef struct pumpctl_params
{
    // What we're aiming for
    int32_t target_pressure_mbar;

    // Rate at which we approach target pressure
    int32_t ramp_rate_mbar_per_sec;

    // Loop gain x sf (PWM count / mbar) * sf
    int32_t gain;

    // Derivative T ms
    int32_t deriv_time;

    // Intergral T ms
    int32_t integ_time;

    // Integrator limits before scaling
    int32_t int_limit;

    // Feed forward gain, (PWM count / mBar) * sf
    int32_t feed_forward_gain;

    // 2nd order filter applied to measured pressure
    int32_t pressure_lpf_cutoff_hz;

} pumpctl_params_t;

void pumpctl_init(pumpctl_status_notify_fn_t status_fn);

void pumpctl_set_pressure(int32_t target_mbar);
void pumpctl_set_deriv_time(int32_t gain_scaled);
void pumpctl_set_integ_time(int32_t integ_time);
void pumpctl_set_int_limit(int32_t int_limit);
void pumpctl_set_gain(int32_t gain_scaled);
void pumpctl_set_ramprate(int32_t mbar_per_sec);
void pumpctl_set_feed_forward_gain(int32_t feed_forward_gain);
void pumpctl_set_pressure_lpf_cutoff_hz(int32_t cutoff_hz);

char const * pumpctl_state_to_str(pumpctl_state_t state);

void pumpctl_get_status(pumpctl_data_t * data_out);
void pumpctl_get_params(pumpctl_params_t *params);

// Queued read of ADC1
int32_t pumpctl_read_adc(uint32_t channel);

#endif // PUMP_CONTROL_H
