#include "lp_filter.h"

#include "esp_log.h"
#include "string.h"


#define BASE_SHIFT (29)
#define RADIX_BITS (3)

lpf_params_t cooked_params[] =
{
    {   507178  ,   1026066113  ,   -491223911  },
    {   1944375 ,   978551887   ,   -449458477  },
    {   4198442 ,   931323806   ,   -411246662  },
    {   7172166 ,   884473343   ,   -376291094  },
    {   10782175    ,   838065198   ,   -344322985  },
    {   14957098    ,   792142531   ,   -315100012  },
    {   14957098    ,   792142531   ,   -315100012  },
    {   19635965    ,   746731218   ,   -288404168  },
    {   24766823    ,   701843306   ,   -264039684  },
    {   30305537    ,   657479811   ,   -241831049  },
    {   36214774    ,   613632985   ,   -221621170  },
};

#define MAX_FF (sizeof(cooked_params)/sizeof(cooked_params[0]))

void lpf_create(lpf_state_t * state, int32_t ff_percent, int32_t input_max_val)
{

    if (!state || ff_percent <= 0 || ff_percent > MAX_FF)
    {
        ESP_LOGE(__func__, "Unable to create filter: state=%p, ff_percent=%d, input_max_val=%d", state, ff_percent, input_max_val);
        return;
    }
    memset(state, 0, sizeof(*state));

    lpf_params_t const * params = &cooked_params[ff_percent - 1];

    int32_t req_bits;
    for (req_bits = 1; input_max_val >> req_bits; ++req_bits);

    state->params.gain = (params->gain + (1 << (req_bits - 1)) ) >> req_bits;

    req_bits += RADIX_BITS;

    state->params.coeff1 = (params->coeff1 + (1 << (req_bits - 1)) ) >> req_bits;
    state->params.coeff2 = (params->coeff2 + (1 << (req_bits - 1)) ) >> req_bits;
    state->shift = BASE_SHIFT - req_bits;

    ESP_LOGI(__func__, "Created LP filter: ff_percent=%d, input_max_val=%d, required bits %d, scaling_fact=%d, gain=%d, coeff1=%d, coeff2=%d",
                            ff_percent,
                            input_max_val,
                            req_bits,
                            1 << state->shift,
                            state->params.gain,
                            state->params.coeff1,
                            state->params.coeff2);
}

int32_t lpf_process(lpf_state_t * state, int32_t sample)
{
    state->in[0] = state->in[1];
    state->in[1] = state->in[2];
    state->in[2] = sample;
    state->out[0] = state->out[1];
    state->out[1] = state->out[2];
    state->out[2] =  ( (state->in[0] + 2 * state->in[1] + state->in[2]) * state->params.gain
                     + ( state->params.coeff2 * state->out[0])
                     + ( state->params.coeff1 * state->out[1]) )  >> state->shift;

    return state->out[2] >> RADIX_BITS;
}
