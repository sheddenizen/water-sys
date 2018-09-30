#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "pump_control.h"

#include "stdint.h"
#include "stdbool.h"
#include <sys/time.h>

#define MAX_JOB_QUEUE_SIZE ( 12 )

typedef struct
{
    uint16_t run_time_secs;
    uint16_t solenoid;
    uint16_t pressure_mbar;

} sch_job_details_t;

typedef enum
{
    SCH_IDLE,
    SCH_RUNNING,
    SCH_FINISHING,
    SCH_DEPRESSURIZING
} sch_state_t;

typedef struct
{
    struct timeval timestamp;
    uint16_t job_queue_size;
    uint16_t current_job_remaining_secs;
    sch_state_t state;
    char const * last_job_result;
    sch_job_details_t job_queue[MAX_JOB_QUEUE_SIZE];
} sch_status_t;

typedef void (*sch_status_update_fn_t)(sch_status_t const *status);

void sch_init(sch_status_update_fn_t update_fn);

void sch_add_watering_job(uint16_t solenoid, uint16_t run_time_secs, uint16_t pressure_mbar);
void sch_clear_queue();
void sch_on_pump_state_change(pumpctl_state_t new_state);
char const * sch_state_to_str(sch_state_t state);


#endif // SCHEDULER_H
