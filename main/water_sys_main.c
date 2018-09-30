#include <stdio.h>
#include <string.h>
#include <time.h>
#include "esp_system.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"

#include "sdkconfig.h"

#include "mqtt_client.h"
#include "app_io.h"
#include "pump_control.h"
#include "scheduler.h"

#include "apps/sntp/sntp.h"
#include "freertos/timers.h"


/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

static const char *TAG = "main";

int unstable_rate_divider = 5;

static uint16_t default_duration_secs = 300;
static uint16_t pressures_mbar[NUM_SOLENOIDS] = { 750, 500, 500, 500, 500, 500 };

static const mqtt_size_t payload_size = sizeof(union
{
    pumpctl_data_t m1;
});

static inline int32_t thousanths(int32_t in)
{
    return (in >= 0 ? in : -in) % 1000;
}

static inline int32_t tenths(int32_t in)
{
    return (in >= 0 ? in : -in) % 10;
}


static uint16_t extract_solenoid(char const *topic, mqtt_size_t topic_len)
{
    for (int n = topic_len -1; n >= 0; --n)
    {
        if (topic[n] == '/')
        {
            uint16_t soln = NUM_SOLENOIDS;
            sscanf(&topic[n+1], "%hu", &soln);
            return soln;
        }
    }
    return NUM_SOLENOIDS;
}

static void set_duration(char const *topic, mqtt_size_t topic_len, char const *msg, mqtt_size_t msg_len)
{
    ESP_LOGI(TAG, "Rx'd: %.*s: 't%.*s'", topic_len, topic, msg_len, msg);

    sscanf(msg, "%hd", &default_duration_secs);
}

static void set_pressure(char const *topic, mqtt_size_t topic_len, char const *msg, mqtt_size_t msg_len)
{
    uint16_t soln = extract_solenoid(topic, topic_len);
    if (soln >= NUM_SOLENOIDS)
    {
        ESP_LOGW(TAG, "Cant set pressure to %s, unknown solenoid in topic: %s", msg, topic);
        return;
    }

    uint16_t pressure_mbar = 0;
    if (sscanf(msg, "%hu", &pressure_mbar))
    {
        ESP_LOGI(TAG, "Set pressure to %hu for solenoid %hu", pressure_mbar, soln);
        pressures_mbar[soln] = pressure_mbar;
    }
    else
    {
        ESP_LOGW(TAG, "Cant set pressure to %s for solenoid %hu", msg, soln);
    }
}

static void clear_queue(char const *topic, mqtt_size_t topic_len, char const *msg, mqtt_size_t msg_len)
{
    ESP_LOGI(TAG, "Rx'd: %.*s: 't%.*s'", topic_len, topic, msg_len, msg);

    sch_clear_queue();
}

static void add_job(char const *topic, mqtt_size_t topic_len, char const *msg, mqtt_size_t msg_len)
{
    ESP_LOGI(TAG, "Rx'd: %.*s: 't%.*s'", topic_len, topic, msg_len, msg);

    uint16_t soln = extract_solenoid(topic, topic_len);
    if (soln >= NUM_SOLENOIDS)
    {
        ESP_LOGW(TAG, "Cant schedule %ss job, unknown solenoid in topic: %s", msg, topic);
        return;
    }

    uint16_t duration_secs = default_duration_secs;
    if (msg_len == 0 || sscanf(msg, "%hu", &duration_secs))
    {
        ESP_LOGI(TAG, "Scheduling job for solenoid %hu for %hu seconds", soln, duration_secs);
        sch_add_watering_job(soln, duration_secs, pressures_mbar[soln]);
    }
    else
    {
        ESP_LOGW(TAG, "Cant schedule job for solenoid %hu, unknown duration %s", soln, msg);
    }
}

static mqtt_subscription_t const subscriptions[] =
{
    { "garden/controller/watersys/courtyard/addJob/+", add_job },
    { "garden/controller/watersys/courtyard/setPressure/+", set_pressure },
    { "garden/controller/watersys/courtyard/setDuration", set_duration },
    { "garden/controller/watersys/courtyard/clearQueue", clear_queue },
};

extern void console_main();

static void start_ntp();

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        mqtt_connection_notify(true);
        start_ntp();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        mqtt_connection_notify(false);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}


static void initialize_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static mqtt_size_t pump_serializer(char const * in_data, mqtt_size_t in_data_size,
                                            char * out_buf, mqtt_size_t out_buf_size)
{
    pumpctl_data_t const *data = (pumpctl_data_t const *)in_data;
    if (in_data_size != sizeof(pumpctl_data_t))
    {
        ESP_LOGE(TAG, "Payload size mismatch, should be %u, is %u", sizeof(pumpctl_data_t), in_data_size);
        return 0;
    }

    struct timeval tv;
    gettimeofday(&tv, NULL);

    time_t sampsecs = 0;
    uint16_t sampmsecs = 0;

    // Don't calculate timestamp unless the time has been set to a sane value
    if (tv.tv_sec > 1535000000)
    {
        // ms portion of sample time
        sampmsecs = data->time_ms % 1000;

        // Seconds portion of sample time
        sampsecs = data->time_ms / 1000;

        // Have bottom bits of time wrapped since sampling?
        sampsecs = (uint16_t)(tv.tv_sec & 63) < sampsecs ? ((tv.tv_sec + 64) & ~63) | sampsecs : (tv.tv_sec& ~63) | sampsecs;
    }

    return snprintf(out_buf, out_buf_size, "{\"unixTime\": %lu.%03u, \"pressuremBar\": %hd, "
                                "\"pumpCurrentmA\": %hd, \"drivevolts\": %hd.%03d, \"supplyvolts\": %hd.%03hd, "
                                "\"tankpercent\": %hd.%d, \"integrator\":%hd, \"setpointmBar\": %hd, \"state\": %hd, "
                                "\"statename\": \"%s\", \"loopMinUs\": %hd,  \"loopMaxUs\": %hd}",
        sampsecs,
        sampmsecs,
        data->pressure_mbar,
        data->current_milliamps,
        data->drive_mv / 1000,
        thousanths(data->drive_mv),
        data->power_supply_mv / 1000,
        thousanths(data->power_supply_mv),
        data->tank_level_percent_x10 / 10,
        tenths(data->tank_level_percent_x10),
        data->integrator_mbar,
        data->setpoint,
        data->state,
        pumpctl_state_to_str(data->state),
        data->loop_min_us,
        data->loop_max_us);
}


static bool pump_status_handler(pumpctl_data_t const *data)
{
    static pumpctl_state_t state;
    static int counter = 0;
    bool send_telem = false;

    if (data->state != state)
    {
        // State has changed, notify scheduler and send telemetry
        state = data->state;
        sch_on_pump_state_change(state);
        send_telem = true;
    }

    // If state unchanged, send telemetry periodically at state dependent rate
    if (!send_telem)
    {
        switch (state)
        {
            case PUMPCTL_STARTING:
            case PUMPCTL_STOPPING:
            case PUMPCTL_UNSTABLE:
                send_telem = counter >= unstable_rate_divider;
                break;
            case PUMPCTL_E_STOP:
            case PUMPCTL_STABLE:
                send_telem = counter >= 100;
                break;
            case PUMPCTL_IDLE:
                send_telem = counter >= 3000;
                break;
        }
    }
    if (send_telem)
    {
        counter = 0;
        mqtt_post_data("garden/controller/watersys/courtyard/pumpctl/state",
                pump_serializer, (char const *)data, sizeof(*data), 1, 0, false);
    }
    ++counter;

    // If we send data return true to reset cumulative stats
    return send_telem;
}


static mqtt_size_t scheduler_serializer(char const * in_data, mqtt_size_t in_data_size,
                                            char * out_buf, mqtt_size_t out_buf_size)
{
    sch_status_t const *status = (sch_status_t const *)in_data;
    if (in_data_size != sizeof(sch_status_t))
    {
        ESP_LOGE(TAG, "Payload size mismatch, should be %u, is %u", sizeof(sch_status_t), in_data_size);
        return 0;
    }

    size_t size = snprintf(out_buf, out_buf_size, "{\"unixTime\": %lu.%03lu, \"state\": \"%s\", "
                                "\"jobQueueSize\": %hu, \"lastJobResult\": \"%s\", "
                                "\"remainingSecs\": %hu, \"currentSolenoid\": %hu, \"queue\": [",
        status->timestamp.tv_sec,
        status->timestamp.tv_usec/1000,
        sch_state_to_str(status->state),
        status->job_queue_size,
        status->last_job_result,
        status->current_job_remaining_secs,
        status->job_queue[0].solenoid);

    for (int n = 0; n < status->job_queue_size && size < (out_buf_size - 3); ++n)
    {
        size += snprintf(&out_buf[size], out_buf_size - size,
                    "%s{\"solenoid\": %hd, \"duration\": %hd, \"pressure\":%hd}",
                    n == 0 ? "" : ",",
                    status->job_queue[n].solenoid,
                    status->job_queue[n].run_time_secs,
                    status->job_queue[n].pressure_mbar);
    }
    if (size < (out_buf_size - 3))
    {
        size += snprintf(&out_buf[size], out_buf_size - size, "]}");
    }
    return size;
}

void scheduler_status_handler(sch_status_t const * status)
{
    mqtt_post_data("garden/controller/watersys/courtyard/scheduler/state",
            scheduler_serializer, (char const *)status, sizeof(*status), 1, 1, true);
}

static void start_ntp()
{
    static bool ntp_started = false;

    if (ntp_started)
    {
        ESP_LOGI(TAG, "Restarting SNTP");
        sntp_stop();
    }
    else
    {
        ESP_LOGI(TAG, "Initializing SNTP");
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "pool.ntp.org");
    }
    sntp_init();

    ntp_started = true;
}

void app_main()
{
    appio_init();
    initialize_nvs();
    mqtt_init(subscriptions, sizeof(subscriptions)/sizeof(subscriptions[0]), payload_size);
    initialise_wifi();
    pumpctl_init(pump_status_handler);
    sch_init(scheduler_status_handler);
    console_main();
}
