#include "scheduler.h"

#include "pump_control.h"
#include "app_io.h"

#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_log.h"


#define MUTEX_TIMEOUT (5000 / portTICK_PERIOD_MS)
#define STATUS_UPDATE_PERIOD_IDLE (60000 / portTICK_PERIOD_MS)
#define STATUS_UPDATE_PERIOD_BUSY (1000 / portTICK_PERIOD_MS)
#define DEPRESSURIZING_TIME (2000 / portTICK_PERIOD_MS)


typedef union
{
    pumpctl_state_t pump_state;
    sch_job_details_t job;
} cmd_payload_t;

typedef void(*cmd_fn_t)(cmd_payload_t const *);

typedef struct
{
    cmd_fn_t cmd_fn;
    cmd_payload_t cmd_payload;
} cmd_t;

static char const estop_msg[] = "Emergency Stop";
static char const abort_msg[] = "User Aborted";
static char const success_msg[] = "Successful";
static char const notrun_msg[] = "Not Applicable";

static QueueHandle_t task_q;
static sch_status_t status;
static TimerHandle_t job_timer;
static sch_status_update_fn_t status_callback_fn;
static pumpctl_state_t pump_state;

static void start_new_job()
{
    sch_job_details_t const * job = &status.job_queue[0];

    ESP_LOGI(__func__, "Starting new job of %hu, solenoid %hu @ %hu mbar for %hu secs",
                        status.job_queue_size, job->solenoid, job->pressure_mbar, job->run_time_secs);

    appio_close_all_solenoids();
    appio_set_solenoid(job->solenoid, true);
    pumpctl_set_pressure(job->pressure_mbar);

    status.state = SCH_RUNNING;
    status.current_job_remaining_secs = job->run_time_secs;

    // Run for the requested time - this should start the timer as well as set the period
    xTimerChangePeriod( job_timer, job->run_time_secs * 1000 / portTICK_PERIOD_MS, 1 );
}

static void finish_current_job()
{
    sch_job_details_t const * job = &status.job_queue[0];

    if (status.job_queue_size == 0)
    {
        ESP_LOGE(__func__, "No jobs remaining?!");
    }

    ESP_LOGI(__func__, "Finishing job, solenoid %hu @ %hu mbar for %hu secs, %hu jobs remaining",
                        job->solenoid, job->pressure_mbar, job->run_time_secs, status.job_queue_size);

    // Start ramping down pressure
    pumpctl_set_pressure(0);
    status.current_job_remaining_secs = 0;

    status.state = SCH_FINISHING;
}

static void complete_current_job()
{
    sch_job_details_t const * job = &status.job_queue[0];

    if (status.job_queue_size == 0)
    {
        ESP_LOGE(__func__, "No jobs remaining?!");
    }

    ESP_LOGI(__func__, "Completing job, solenoid %hu @ %hu mbar for %hu secs, %hu jobs remaining",
                        job->solenoid, job->pressure_mbar, job->run_time_secs, status.job_queue_size);

    --status.job_queue_size;
    for (int n = 0; n < status.job_queue_size; ++n)
    {
        status.job_queue[n] = status.job_queue[n+1];
    }
    status.last_job_result = success_msg;

    if (status.job_queue_size > 0)
    {
        start_new_job();
    }
    else
    {
        ESP_LOGI(__func__, "Finished all jobs, beginning depressuraization");
        status.state = SCH_DEPRESSURIZING;
        xTimerChangePeriod( job_timer, DEPRESSURIZING_TIME, 1 );
    }
}

static void add_watering_job(cmd_payload_t const * payload)
{
    sch_job_details_t const * job = &payload->job;
    bool start_running = false;

    if (MAX_JOB_QUEUE_SIZE == status.job_queue_size)
    {
        ESP_LOGE(__func__, "Unable to add job, solenoid %hu @ %hu for %hu secs, queue full",
                        job->solenoid, job->pressure_mbar, job->run_time_secs);
    }
    else
    {
        status.job_queue[status.job_queue_size] = *job;
        start_running = status.job_queue_size == 0;
        ++status.job_queue_size;

        ESP_LOGI(__func__, "Added job %hu, solenoid %hu @ %hu mbar for %hu secs",
                        status.job_queue_size, job->solenoid, job->pressure_mbar, job->run_time_secs);
    }
    if (start_running)
        start_new_job();
}

static void clear_queue()
{
    // Start ramping down pressure
    pumpctl_set_pressure(0);

    if (status.state != SCH_IDLE)
    {
        ESP_LOGI(__func__, "Abandoning %hu jobs, beginning depressuraization", status.job_queue_size);
        status.state = SCH_DEPRESSURIZING;
        status.job_queue_size = 0;
        xTimerChangePeriod( job_timer, DEPRESSURIZING_TIME, 1 );
    }
    else
    {
        ESP_LOGI(__func__, "Abandoning %hu jobs, already idle?", status.job_queue_size);
    }
}

static void user_clear_queue(cmd_payload_t const * payload)
{

    switch (status.state)
    {
        case SCH_IDLE:
            ESP_LOGI(__func__, "Can't abort, already idle");
            return;
        case SCH_DEPRESSURIZING:
            ESP_LOGI(__func__, "Can't abort, already depressurizing");
            return;
        case SCH_FINISHING:
            if (status.job_queue_size == 0)
            {
                ESP_LOGI(__func__, "Can't abort, already finishing last job");
                return;
            }
        case SCH_RUNNING:
            ESP_LOGI(__func__, "User abort, starting depressurizing");
            status.last_job_result = abort_msg;
            clear_queue();
            break;
    }
}

static void on_pump_state_change(cmd_payload_t const * payload)
{
    pump_state = payload->pump_state;

    switch (pump_state)
    {
        case PUMPCTL_STARTING:
        case PUMPCTL_STOPPING:
        case PUMPCTL_UNSTABLE:
        case PUMPCTL_STABLE:
            break;
        case PUMPCTL_E_STOP:
            // Something bad happened
            clear_queue();
            break;
        case PUMPCTL_IDLE:
            // Pump has ramped down - next job or depressurize
            if (status.state == SCH_FINISHING)
            {
                complete_current_job();
            }
            break;
    }
}

void timer_expire(cmd_payload_t const * payload)
{
    if (status.state == SCH_DEPRESSURIZING)
    {
        ESP_LOGI(__func__, "Depressurizing finished, now idle");
        appio_close_all_solenoids();
        status.state = SCH_IDLE;
    }
    else if (status.state == SCH_RUNNING)
    {
        finish_current_job();
    }
}

void timer_expire_callback( TimerHandle_t xTimer )
{
    cmd_t entry;
    entry.cmd_fn = &timer_expire;

    xQueueSend(task_q, &entry, 10);
}

void scheduler_task(void *arg)
{
    sch_state_t last_state = SCH_IDLE;
    uint16_t last_jobcount = 0;
    uint32_t update_period = STATUS_UPDATE_PERIOD_IDLE;
    while (1)
    {
        bool notify_update = false;
        cmd_t cmd;
        if (pdTRUE == xQueueReceive(task_q, &cmd, update_period))
        {
            cmd.cmd_fn(&cmd.cmd_payload);
            notify_update = last_state != status.state || last_jobcount != status.job_queue_size;
            last_state = status.state;
            last_jobcount = status.job_queue_size;
            update_period = status.state == SCH_IDLE ?
                STATUS_UPDATE_PERIOD_IDLE : STATUS_UPDATE_PERIOD_BUSY;
        }
        else
        {
            status.current_job_remaining_secs = status.job_queue_size == 0 ? 0 :
                (xTimerGetExpiryTime( job_timer ) - xTaskGetTickCount()) * portTICK_PERIOD_MS / 1000 ;
            notify_update = true;
        }
        if (notify_update && status_callback_fn)
        {
            gettimeofday(&status.timestamp, NULL);
            status_callback_fn(&status);
        }
    }
}

void sch_init(sch_status_update_fn_t update_fn)
{
    job_timer = xTimerCreate("job timer", STATUS_UPDATE_PERIOD_IDLE, pdFALSE, NULL, timer_expire_callback);
    task_q = xQueueCreate( 1, sizeof(cmd_t));
    status_callback_fn = update_fn;

    status.last_job_result = notrun_msg;

    ESP_LOGI(__func__, "Starting scheduler task");
    BaseType_t res = xTaskCreate(&scheduler_task, "scheduler", 4096, NULL, 10, NULL);
    if (pdPASS != res)
        ESP_LOGE(__func__, "Failed to start scheduler task, %d", res);
}

void sch_add_watering_job(uint16_t solenoid, uint16_t run_time_secs, uint16_t pressure_mbar)
{
    cmd_t entry;
    sch_job_details_t * job = &entry.cmd_payload.job;

    if (solenoid >= NUM_SOLENOIDS || run_time_secs == 0 || pressure_mbar == 0 || pressure_mbar > 5000 || run_time_secs > 3600)
    {
        ESP_LOGE(__func__, "Rejected job, solenoid, %hu, for %hu secs at %hu mbar, out of range parameter", solenoid, run_time_secs, pressure_mbar);
        return;
    }

    job->solenoid = solenoid;
    job->run_time_secs = run_time_secs;
    job->pressure_mbar = pressure_mbar;

    entry.cmd_fn = &add_watering_job;

    xQueueSend(task_q, &entry, 10);
}

void sch_clear_queue()
{
    cmd_t entry;

    entry.cmd_fn = &user_clear_queue;

    xQueueSend(task_q, &entry, 10);
}

void sch_on_pump_state_change(pumpctl_state_t new_state)
{
    cmd_t entry;
    entry.cmd_payload.pump_state = new_state;

    entry.cmd_fn = &on_pump_state_change;

    xQueueSend(task_q, &entry, 10);
}

char const * sch_state_to_str(sch_state_t state)
{
    static char const *state_names[] =
    {
        "IDLE",
        "RUNNING",
        "FINISHING",
        "DEPRESSURIZING",
    };
    return state_names[state];
}
