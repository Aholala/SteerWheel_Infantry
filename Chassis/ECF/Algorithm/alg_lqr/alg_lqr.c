#include "alg_lqr.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>

static bool alg_lqr_is_finite_array(const float *values, size_t count)
{
    size_t index;
    for (index = 0U; index < count; ++index)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }
    return true;
}

alg_lqr_status_t alg_lqr_init(alg_lqr_t *me, const alg_lqr_config_t *config)
{
    size_t control_index;
    if ((me == NULL) || (config == NULL) || (config->gain_matrix == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U) ||
        (config->state_dimension > (SIZE_MAX / config->control_dimension)) ||
        ((config->control_min == NULL) != (config->control_max == NULL)) ||
        !alg_lqr_is_finite_array(config->gain_matrix,
                                 config->state_dimension * config->control_dimension))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if (config->control_min != NULL)
    {
        for (control_index = 0U; control_index < config->control_dimension; ++control_index)
        {
            if (!isfinite(config->control_min[control_index]) ||
                !isfinite(config->control_max[control_index]) ||
                (config->control_min[control_index] >= config->control_max[control_index]))
            {
                return ALG_LQR_STATUS_OUT_OF_RANGE;
            }
        }
    }
    me->config = *config;
    me->is_initialized = true;
    return ALG_LQR_STATUS_OK;
}

alg_lqr_status_t alg_lqr_update(const alg_lqr_t *me, const float *reference,
                                const float *state, const float *feedforward, float *output)
{
    size_t control_index;
    size_t state_index;
    float control;
    float state_error;

    if ((me == NULL) || (state == NULL) || (output == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_LQR_STATUS_NOT_INITIALIZED;
    }
    if (!alg_lqr_is_finite_array(state, me->config.state_dimension) ||
        ((reference != NULL) &&
         !alg_lqr_is_finite_array(reference, me->config.state_dimension)) ||
        ((feedforward != NULL) &&
         !alg_lqr_is_finite_array(feedforward, me->config.control_dimension)))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    for (control_index = 0U; control_index < me->config.control_dimension; ++control_index)
    {
        control = (feedforward != NULL) ? feedforward[control_index] : 0.0F;
        for (state_index = 0U; state_index < me->config.state_dimension; ++state_index)
        {
            state_error = state[state_index] -
                          ((reference != NULL) ? reference[state_index] : 0.0F);
            control -= me->config.gain_matrix[(control_index * me->config.state_dimension) +
                                              state_index] *
                       state_error;
        }
        if (!isfinite(control))
        {
            return ALG_LQR_STATUS_NUMERICAL_ERROR;
        }
        if (me->config.control_min != NULL)
        {
            if (control < me->config.control_min[control_index])
            {
                control = me->config.control_min[control_index];
            }
            else if (control > me->config.control_max[control_index])
            {
                control = me->config.control_max[control_index];
            }
        }
        output[control_index] = control;
    }
    return ALG_LQR_STATUS_OK;
}
