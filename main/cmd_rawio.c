/* Console — Manual IO control

*/

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "cmd_decl.h"
#include "freertos/FreeRTOS.h"
#include "app_io.h"
#include "pump_control.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** Arguments used by 'setsoln' function */
static struct {
    struct arg_int *soln;
    struct arg_int *state;
    struct arg_end *end;
} setsoln_args;

/** Arguments used by 'setpump' function */
static struct {
    struct arg_str *param;
    struct arg_int *value;
    struct arg_end *end;
} setpump_args;

/** Arguments used by 'status' function */
static struct {
    struct arg_end *end;
} no_args;


// Sneaky and dirty - remove or replace
extern int unstable_rate_divider;

static int cons_setsoln(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &setsoln_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, setsoln_args.end, argv[0]);
        return 1;
    }
    ESP_LOGI(__func__, "Set soln %d to %d",
            setsoln_args.soln->ival[0], setsoln_args.state->ival[0]);

    appio_set_solenoid(setsoln_args.soln->ival[0],
                           setsoln_args.state->ival[0]);

    return 0;
}

static void set_sample_rate_divider(int32_t divider)
{
    unstable_rate_divider = divider;
}

static int cons_setpump(int argc, char** argv)
{
    struct param_tbl_struct { char param; char const * full; void (*setter)(int32_t); };
    struct param_tbl_struct const param_tbl[] =
    {
        { 't', "Target Pressure", &pumpctl_set_pressure },
        { 'p', "Overall gain", &pumpctl_set_gain },
        { 'i', "Integration Time", &pumpctl_set_integ_time },
        { 'd', "Derivative Time", &pumpctl_set_deriv_time },
        { 'l', "Integrator Limit", &pumpctl_set_int_limit },
        { 'r', "Ramp Rate", &pumpctl_set_ramprate },
        { 'f', "Feed Forward Gain", &pumpctl_set_feed_forward_gain },
        { 'c', "Pressure Filter Cutoff Hz", &pumpctl_set_pressure_lpf_cutoff_hz },
        { 's', "Sample Period, Cycles", &set_sample_rate_divider },
    };
    struct param_tbl_struct const * param_match = NULL;

    int nerrors = arg_parse(argc, argv, (void**) &setpump_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, setpump_args.end, argv[0]);
        return 1;
    }

    for (int n = 0 ; n < (sizeof(param_tbl)/sizeof(param_tbl[0])); ++n)
    {
        if (setpump_args.param->sval[0][0] == param_tbl[n].param)
        {
            param_match = &param_tbl[n];
            break;
        }
    }
    if (param_match)
    {
        ESP_LOGI(__func__, "Set pump %s to %d",
                param_match->full, setpump_args.value->ival[0]);
        param_match->setter(setpump_args.value->ival[0]);
    }
    else
    {
        ESP_LOGE(__func__, "Unknown pump parameter %s (cant set to %d)",
                setpump_args.param->sval[0], setpump_args.value->ival[0]);
    }

    return 0;
}


static int cons_status(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &no_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, no_args.end, argv[0]);
        return 1;
    }
    ESP_LOGI(__func__, "Get System Status:\nPressure: %d mBar\nPump current: %d mA\nTank: %d",
                                                            appio_get_outlet_pressure_mbar(),
                                                            appio_get_pump_current_milliamps(),
                                                            appio_get_tank_level_percent_x10());
    for (int n = 0; n < NUM_SOLENOIDS; ++n)
        ESP_LOGI(__func__, "Solenoid %d: %s", n, appio_get_solenoid(n) ? "open" : "closed");
        ESP_LOGI(__func__, "Pump drive %u", appio_get_pump_drive());
        ESP_LOGI(__func__, "%s open", appio_any_solenoid_open() ? "At least one solenoid" : "No solenoids");
    pumpctl_data_t data;
    pumpctl_get_status(&data);

    ESP_LOGI(__func__, "Pressure mBar: %hd, Pump Current mA: %hd, Drive volts: %hd.%hd\n"
                        "Supply volts: %hd.%hd, Tank %%: %hd.%hd\n"
                        "Integrator mBar:%hd, Setpoint mBar: %hd, state: %s\n"
                        "Loop min us %hu, Loop Max us, %hu",
        data.pressure_mbar,
        data.current_milliamps,
        data.drive_mv / 1000,
        data.drive_mv % 1000,
        data.power_supply_mv / 1000,
        data.power_supply_mv % 1000,
        data.tank_level_percent_x10 / 10,
        data.tank_level_percent_x10 % 10,
        data.integrator_mbar,
        data.setpoint,
        pumpctl_state_to_str(data.state),
        data.loop_min_us,
        data.loop_max_us);

    pumpctl_params_t params;

    pumpctl_get_params(&params);

    ESP_LOGI(__func__, "Target Pressure, mBar: %d, Ramp Rate, mBar/s: %d\n"
                        "Loop Gain x 1000: %d, Integral Time ms: %d, Deriv Time ms: %d\n"
                        "Integrator Limit: %d, Feed Forward Gain: %d, Pressure LP Filter, Hz: %d, sample divider: %d",
        params.target_pressure_mbar,
        params.ramp_rate_mbar_per_sec,
        params.gain,
        params.integ_time,
        params.deriv_time,
        params.int_limit,
        params.feed_forward_gain,
        params.pressure_lpf_cutoff_hz,
        unstable_rate_divider);

    return 0;
}


static int cons_reset(int argc, char** argv)
{
    int nerrors = arg_parse(argc, argv, (void**) &no_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, no_args.end, argv[0]);
        return 1;
    }
    ESP_LOGE(__func__, "Death is sweet");

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    esp_restart();

    return 0;
}

void register_rawio()
{
    setsoln_args.soln = arg_int1(NULL, NULL, "<soln>", "GPIO to set");
    setsoln_args.state = arg_int1(NULL, NULL, "<state>", "1-High, 0-Low");
    setsoln_args.end = arg_end(2);

    const esp_console_cmd_t setsoln_cmd = {
        .command = "sol",
        .help = "Set state of solenoid",
        .hint = NULL,
        .func = &cons_setsoln,
        .argtable = &setsoln_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&setsoln_cmd) );

    setpump_args.param = arg_str1(NULL, NULL, "<param>", "Parameter to change, t:- target pressure, p - overall gain, "
                        "i - integration time, d - derivative time, r - ramp rate, "
                        "f - feed forward gain c - LP filter Hz, s - unstable sample rate divider");
    setpump_args.value = arg_int1(NULL, NULL, "<value>", "value");
    setpump_args.end = arg_end(2);

    const esp_console_cmd_t setpump_cmd = {
        .command = "pump",
        .help = "Set pump control parameter",
        .hint = NULL,
        .func = &cons_setpump,
        .argtable = &setpump_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&setpump_cmd) );

    no_args.end = arg_end(2);

    const esp_console_cmd_t status_cmd = {
        .command = "stat",
        .help = "Show system status",
        .hint = NULL,
        .func = &cons_status,
        .argtable = &no_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&status_cmd) );

    const esp_console_cmd_t reset_cmd = {
        .command = "rst",
        .help = "Reset system",
        .hint = NULL,
        .func = &cons_reset,
        .argtable = &no_args
    };

    ESP_ERROR_CHECK( esp_console_cmd_register(&reset_cmd) );
}
