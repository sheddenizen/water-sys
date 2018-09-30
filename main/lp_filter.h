#ifndef LP_FILTER_H
#define LP_FILTER_H

#include "stdint.h"

#define ORDER (2)


typedef struct
{
    int32_t gain;
    int32_t coeff1;
    int32_t coeff2;
} lpf_params_t;

typedef struct
{
    lpf_params_t params;
    int32_t shift;
    int32_t in[ORDER + 1];
    int32_t out[ORDER + 1];
} lpf_state_t;


void lpf_create(lpf_state_t * state, int32_t ff_percent, int32_t input_max_val);
int32_t lpf_process(lpf_state_t * state, int32_t sample);

#endif // LP_FILTER_H
