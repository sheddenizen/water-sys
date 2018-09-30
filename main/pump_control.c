#include "pump_control.h"

#include <sys/time.h>
#include "app_io.h"
#include "lp_filter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "pumpctl";

// Limit pump drive until things are under control
#define PUMP_DRIVE_CLAMP ( 3 * MAX_PUMP_DRIVE / 4 )

// Loop interval
#define LOOP_PERIOD_MS ( 10 )

#define LOOP_PERIOD_TICKS (10 / portTICK_PERIOD_MS)

// Times are expressed in ms - need to scale where appropriate
#define TSCALE (1000 / LOOP_PERIOD_MS)

// If minimal fluctuation in error over this time, we consider operation stable
#define STABILIZATION_TIME_MS ( 5000 )

#define SUPPLY_MILLI_VOLTS (12000)

// More-or-less the limit of the pressure sensor, not actually achievable by the pump so could be wound back a bit
#define MAX_PRESSURE (10000)

// Filter params for tank level
#define MAX_TANK_LEVEL (1500)
#define TANK_LPF_HZ (1)

// Frequency as a percentage of sample (loop) frequency
// 100% x freq / loop freq = the number you first thought of
#define FREQ_HZ_TO_PERCENT_FF(f)  (100 * LOOP_PERIOD_MS * (f) / 1000)


// Command payload - at present all parameters are signed ints
typedef union
{
    int32_t ctrl_param;
} cmd_payload_t;

typedef void(*cmd_fn_t)(cmd_payload_t const *);

typedef struct
{
    cmd_fn_t cmd_fn;
    cmd_payload_t cmd_payload;
} cmd_t;

// Scaling applied to controller gain term
#define KSCALE 1000;
static QueueHandle_t task_q;

// What we're aiming for
static int32_t target_pressure_mbar = 0;

// Rate at which we approach target pressure
static int32_t ramp_rate_mbar_per_sec = 500;

// Loop gain x sf (PWM count / mbar) * sf
static int32_t gain = 600;

// Derivative T ms
static int32_t deriv_time = 5;

// Intergral T ms
static int32_t integ_time = 1000;

// Integrator limits before scaling
static int32_t int_limit = 1000000;

// Feed forward gain, (PWM count / mBar) * sf
static int32_t feed_forward_gain = 500;

// Filter applied to pressure sensor
static int32_t pressure_lpf_cutoff_hz = 2;

static lpf_state_t pressure_lpf_state;

static lpf_state_t tank_lpf_state;

// Loop execution time
static uint32_t loop_duration_us = 0;

// Current sensor and PID state
static pumpctl_data_t measurements;

// Input value for controller - ramps up or down to
static int32_t setpoint_mbar;

// Integrator - raw cumulative sum of error
static int32_t int_val = 0;

// Remaining loops before we consider the system stable
static uint32_t stable_cycle_count;

// Callback for status update
static pumpctl_status_notify_fn_t status_callback_fn;

// Command queue function generator to save some typing
#define SET_PARAM_FN(param) \
static void set_## param(cmd_payload_t const *payload) \
{ \
    param = payload->ctrl_param; \
}

// Set all the things
SET_PARAM_FN(gain)
SET_PARAM_FN(deriv_time)
SET_PARAM_FN(integ_time)
SET_PARAM_FN(int_limit)
SET_PARAM_FN(target_pressure_mbar)
SET_PARAM_FN(ramp_rate_mbar_per_sec)
SET_PARAM_FN(feed_forward_gain)

static void set_pressure_lpf_cutoff_hz(cmd_payload_t const *payload)
{
    pressure_lpf_cutoff_hz = payload->ctrl_param;
    // Recreate filter
    lpf_create(&pressure_lpf_state, FREQ_HZ_TO_PERCENT_FF(pressure_lpf_cutoff_hz), MAX_PRESSURE);
}

static void collect_measurements()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    measurements.time_ms = (uint16_t)(tv.tv_sec & 63) * 1000 + (uint16_t)(tv.tv_usec / 1000);
    measurements.current_milliamps = (int16_t)appio_get_pump_current_milliamps();
    measurements.power_supply_mv = (int16_t)appio_get_supply_voltage_millivolts();
    measurements.tank_level_percent_x10 = (int16_t)lpf_process(&tank_lpf_state, appio_get_tank_level_percent_x10());
    measurements.pressure_mbar = (int16_t)lpf_process(&pressure_lpf_state, appio_get_outlet_pressure_mbar());
    if (measurements.loop_min_us > loop_duration_us || measurements.loop_min_us == 0)
        measurements.loop_min_us = loop_duration_us;
    // hijack param
    //    measurements.loop_min_us = raw_pressure;
    if (measurements.loop_max_us < loop_duration_us)
        measurements.loop_max_us = loop_duration_us;
//    if (measurements.loop_max_us < raw_pressure)
//        measurements.loop_max_us = raw_pressure;
}

static bool safety_check_fail()
{
    // TODO: More chex
    // Don't run pump unless at least one solenoid open
    return !appio_any_solenoid_open();
}

static void adjust_setpoint()
{

    // If somthing is out of range, force zero target pressure
    if (safety_check_fail() && setpoint_mbar != 0)
    {
        measurements.state = PUMPCTL_E_STOP;
        setpoint_mbar = 0;
        target_pressure_mbar = 0;
    }
    if (setpoint_mbar == target_pressure_mbar)
        return;

    static int32_t ramp_residual = 0;
    int32_t ramp_increment_x1000 = ramp_residual + ramp_rate_mbar_per_sec * LOOP_PERIOD_MS;
    ramp_residual = ramp_increment_x1000 % 1000;

    if ( setpoint_mbar < target_pressure_mbar)
    {
        if (setpoint_mbar == 0)
        {
            // We are transitioning from off to on, reset the integrator
            int_val = 0;
        }
        setpoint_mbar += ramp_increment_x1000 / 1000;
        if ( setpoint_mbar >= target_pressure_mbar)
        {
            setpoint_mbar = target_pressure_mbar;
            measurements.state = PUMPCTL_UNSTABLE;
        }
        else
            measurements.state = PUMPCTL_STARTING;
    }
    else
    {
        setpoint_mbar -= ramp_increment_x1000 / 1000;
        if ( setpoint_mbar <= target_pressure_mbar)
        {
            setpoint_mbar = target_pressure_mbar;
            measurements.state = setpoint_mbar == 0 ? PUMPCTL_IDLE : PUMPCTL_STABLE;
        }
        else
            measurements.state = PUMPCTL_STOPPING;
    }
    stable_cycle_count = 0;
}

static void pump_task(void * arg)
{
    static int32_t last_error = 0;

    TickType_t ticks = xTaskGetTickCount();

    ESP_LOGI(TAG, "Stack remaining for task '%s' is %d bytes", pcTaskGetTaskName(NULL), uxTaskGetStackHighWaterMark(NULL));

    while (1)
    {
        uint32_t loop_start_us = (uint32_t)esp_timer_get_time();

        collect_measurements();

        adjust_setpoint();

        measurements.setpoint = (int16_t)setpoint_mbar;

        int32_t error = setpoint_mbar - measurements.pressure_mbar;

        // Change in error
        int32_t deriv = error - last_error;

        // Accumulate error and apply limits
        int_val += error;

        if (int_val > int_limit || int_val < -int_limit)
        {
            // Integrator limit hit, clamp and signal instability
            stable_cycle_count = 0;
            int_val = int_val > 0 ? int_limit : -int_limit;
        }

        // Report integrator in terms of error (pressure) after integration time
        measurements.integrator_mbar = LOOP_PERIOD_MS * int_val / integ_time;

        // "Standard" PID formula with scaling to compensate for loop interval
        int32_t drive = gain * (error
                                + LOOP_PERIOD_MS * int_val / integ_time
                                + deriv_time * deriv / LOOP_PERIOD_MS ) / KSCALE;

        // Add feed forward
        drive += setpoint_mbar * feed_forward_gain / KSCALE;

        if (drive > PUMP_DRIVE_CLAMP)
        {
            // Overshot - clamp output and hold integrator
            drive = PUMP_DRIVE_CLAMP;
            int_val -= error;
            // appio_set_blinkenlight(true);
            stable_cycle_count = 0;
        }
        if (drive < 0 || setpoint_mbar == 0)
        {
            // Undershot (clamp otherwise bad things will happen), or stop dead in emergency
            // or it's simply off and we want to avoid creep if pressure is negative
            drive = 0;
            int_val -= error;
        }
        measurements.drive_mv = SUPPLY_MILLI_VOLTS * drive / MAX_PUMP_DRIVE;

        last_error = error;

        // TODO: Check and criteria for instability
        if (stable_cycle_count == STABILIZATION_TIME_MS / LOOP_PERIOD_MS)
        {
            if (measurements.state == PUMPCTL_UNSTABLE)
            {
                measurements.state = setpoint_mbar == 0 ? PUMPCTL_IDLE : PUMPCTL_STABLE;
            }
        }
        else
        {
            if (stable_cycle_count == 0 && measurements.state == PUMPCTL_STABLE)
            {
                measurements.state = PUMPCTL_UNSTABLE;
            }
            ++stable_cycle_count;
        }

        // Actually get the pump to do something
        appio_set_pump_drive(drive);

        if (status_callback_fn && status_callback_fn(&measurements))
        {
            measurements.loop_max_us = 0;
            measurements.loop_min_us = 0;
        }

        // Next time round the loop
        ticks += LOOP_PERIOD_TICKS;

        loop_duration_us = (uint32_t)esp_timer_get_time() - loop_start_us;

        int32_t ticks_remaining;
        while( 0 < (ticks_remaining = ticks - xTaskGetTickCount()) )
        {
            cmd_t cmd;
            if (pdTRUE == xQueueReceive(task_q, &cmd, ticks_remaining))
            {
                cmd.cmd_fn(&cmd.cmd_payload);
            }
        }
        if (ticks_remaining < -((int32_t)LOOP_PERIOD_TICKS - 1))
        {
            // Something's taking longer than it should have - catch up and try to tell someone
            ticks -= ticks_remaining;
            appio_set_blinkenlight(true);
        }
        else
        {
            appio_set_blinkenlight(false);
        }
    }
    ESP_LOGE(TAG, "Task '%s' exiting!", pcTaskGetTaskName(NULL));

}

void pumpctl_init(pumpctl_status_notify_fn_t status_fn)
{
    status_callback_fn = status_fn;

    task_q = xQueueCreate( 1, sizeof(cmd_t));

    // Create low pass filter to filter pressure ripple
    lpf_create(&pressure_lpf_state, FREQ_HZ_TO_PERCENT_FF(pressure_lpf_cutoff_hz), MAX_PRESSURE);

    // Create low pass filter to filter tank level
    lpf_create(&tank_lpf_state, FREQ_HZ_TO_PERCENT_FF(TANK_LPF_HZ), MAX_TANK_LEVEL);

    ESP_LOGI(TAG, "Starting pump_ctl task");
    BaseType_t res = xTaskCreate(&pump_task, "pump_ctl", 4096, NULL, 10, NULL);
    if (pdPASS != res)
        ESP_LOGE(TAG, "Failed to start pump_ctl task, %d", res);
}

static void queue_set_param(cmd_fn_t fn, int32_t value)
{
    cmd_t entry;
    entry.cmd_fn = fn;
    entry.cmd_payload.ctrl_param = value;

    xQueueSend(task_q, &entry, 10);
}

void pumpctl_set_pressure(int32_t pressure_mbar)
{
    queue_set_param(set_target_pressure_mbar, pressure_mbar);
}

void pumpctl_set_deriv_time(int32_t deriv_time)
{
    queue_set_param(set_deriv_time, deriv_time);
}

void pumpctl_set_integ_time(int32_t integ_time)
{
    queue_set_param(set_integ_time, integ_time);
}

void pumpctl_set_int_limit(int32_t int_limit)
{
    queue_set_param(set_int_limit, int_limit);
}

void pumpctl_set_gain(int32_t gain_scaled)
{
    queue_set_param(set_gain, gain_scaled);
}

void pumpctl_set_ramprate(int32_t mbar_per_sec)
{
    queue_set_param(set_ramp_rate_mbar_per_sec, mbar_per_sec);
}

void pumpctl_set_feed_forward_gain(int32_t feed_forward_gain)
{
    queue_set_param(set_feed_forward_gain, feed_forward_gain);
}

void pumpctl_set_pressure_lpf_cutoff_hz(int32_t cutoff_hz)
{
    queue_set_param(set_pressure_lpf_cutoff_hz, cutoff_hz);
}

char const * pumpctl_state_to_str(pumpctl_state_t state)
{
    static char const *state_names[] =
    {
        "IDLE",
        "STARTING",
        "STOPPING",
        "UNSTABLE",
        "STABLE",
        "E_STOP"
    };
    return state_names[state];
}

// TODO: Thread safe version
void pumpctl_get_status(pumpctl_data_t * data_out)
{
    if (!data_out)
        return;

    *data_out = measurements;
}

void pumpctl_get_params(pumpctl_params_t *params)
{
    if (!params)
        return;

    params->target_pressure_mbar = target_pressure_mbar;
    params->ramp_rate_mbar_per_sec = ramp_rate_mbar_per_sec;
    params->gain = gain;
    params->deriv_time = deriv_time;
    params->integ_time = integ_time;
    params->int_limit = int_limit;
    params->feed_forward_gain = feed_forward_gain;
    params->pressure_lpf_cutoff_hz = pressure_lpf_cutoff_hz;
}
