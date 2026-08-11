#include "alg_pid.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define ALG_PID_TWO_PI_F 6.28318530717958647692F

static float alg_pid_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static alg_pid_status_t alg_pid_validate_config(const alg_pid_config_t *config)
{
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(config->proportional_gain) || !isfinite(config->integral_gain) ||
        !isfinite(config->derivative_gain) ||
        !isfinite(config->derivative_filter_cutoff_hz) ||
        (config->derivative_filter_cutoff_hz < 0.0F) || !isfinite(config->integral_min) ||
        !isfinite(config->integral_max) || (config->integral_min > config->integral_max) ||
        !isfinite(config->output_min) || !isfinite(config->output_max) ||
        (config->output_min >= config->output_max))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    return ALG_PID_STATUS_OK;
}

alg_pid_status_t alg_pid_config_init(alg_pid_config_t *config)
{
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    *config = (alg_pid_config_t){
        .proportional_gain = 0.0F,
        .integral_gain = 0.0F,
        .derivative_gain = 0.0F,
        .derivative_filter_cutoff_hz = 0.0F,
        .integral_min = -FLT_MAX,
        .integral_max = FLT_MAX,
        .output_min = -FLT_MAX,
        .output_max = FLT_MAX,
        .derivative_on_measurement = true,
    };
    return ALG_PID_STATUS_OK;
}

alg_pid_status_t alg_pid_init(alg_pid_t *me, const alg_pid_config_t *config)
{
    alg_pid_status_t status;
    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    status = alg_pid_validate_config(config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }
    *me = (alg_pid_t){0};
    me->config = *config;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

alg_pid_status_t alg_pid_reset(alg_pid_t *me, float measurement, float initial_output)
{
    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(measurement) || !isfinite(initial_output))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    me->terms = (alg_pid_terms_t){0};
    me->terms.integral = alg_pid_clamp(initial_output, me->config.integral_min,
                                       me->config.integral_max);
    me->terms.unsaturated_output = me->terms.integral;
    me->terms.output = alg_pid_clamp(me->terms.integral, me->config.output_min,
                                     me->config.output_max);
    me->previous_error = 0.0F;
    me->previous_measurement = measurement;
    me->filtered_derivative = 0.0F;
    me->has_previous_sample = true;
    return ALG_PID_STATUS_OK;
}

alg_pid_status_t alg_pid_update(alg_pid_t *me, float setpoint, float measurement,
                                float delta_time_s, float *output)
{
    const alg_pid_input_t input = {
        .setpoint = setpoint,
        .measurement = measurement,
        .velocity_feedforward = 0.0F,
        .acceleration_feedforward = 0.0F,
        .additional_feedforward = 0.0F,
        .delta_time_s = delta_time_s,
    };
    return alg_pid_update_advanced(me, &input, output);
}

alg_pid_status_t alg_pid_update_advanced(alg_pid_t *me, const alg_pid_input_t *input,
                                         float *output)
{
    float error;
    float proportional;
    float integral;
    float derivative_signal = 0.0F;
    float filtered_derivative;
    float derivative;
    float unsaturated_output;
    float saturated_output;
    float smoothing_factor;
    bool saturation_pushes_with_error;

    if ((me == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input->setpoint) || !isfinite(input->measurement) ||
        !isfinite(input->velocity_feedforward) ||
        !isfinite(input->acceleration_feedforward) ||
        !isfinite(input->additional_feedforward) || !isfinite(input->delta_time_s) ||
        (input->delta_time_s <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    error = input->setpoint - input->measurement;
    proportional = me->config.proportional_gain * error;
    integral = alg_pid_clamp(me->terms.integral +
                                 (me->config.integral_gain * error * input->delta_time_s),
                             me->config.integral_min, me->config.integral_max);

    if (me->has_previous_sample)
    {
        derivative_signal = me->config.derivative_on_measurement
                                ? -(input->measurement - me->previous_measurement) /
                                      input->delta_time_s
                                : (error - me->previous_error) / input->delta_time_s;
    }
    filtered_derivative = derivative_signal;
    if (me->has_previous_sample && (me->config.derivative_filter_cutoff_hz > 0.0F))
    {
        smoothing_factor = (ALG_PID_TWO_PI_F * me->config.derivative_filter_cutoff_hz *
                            input->delta_time_s) /
                           (1.0F + (ALG_PID_TWO_PI_F *
                                    me->config.derivative_filter_cutoff_hz *
                                    input->delta_time_s));
        filtered_derivative = me->filtered_derivative +
                              smoothing_factor * (derivative_signal - me->filtered_derivative);
    }
    derivative = me->config.derivative_gain * filtered_derivative;
    me->terms.feedforward = input->velocity_feedforward + input->acceleration_feedforward +
                            input->additional_feedforward;

    unsaturated_output = proportional + integral + derivative + me->terms.feedforward;
    saturated_output = alg_pid_clamp(unsaturated_output, me->config.output_min,
                                     me->config.output_max);
    saturation_pushes_with_error =
        ((unsaturated_output > me->config.output_max) && (error > 0.0F)) ||
        ((unsaturated_output < me->config.output_min) && (error < 0.0F));
    if (saturation_pushes_with_error)
    {
        integral = me->terms.integral;
        unsaturated_output = proportional + integral + derivative + me->terms.feedforward;
        saturated_output = alg_pid_clamp(unsaturated_output, me->config.output_min,
                                         me->config.output_max);
    }
    if (!isfinite(unsaturated_output) || !isfinite(saturated_output))
    {
        return ALG_PID_STATUS_NUMERICAL_ERROR;
    }

    me->terms.proportional = proportional;
    me->terms.integral = integral;
    me->terms.derivative = derivative;
    me->terms.unsaturated_output = unsaturated_output;
    me->terms.output = saturated_output;
    me->previous_error = error;
    me->previous_measurement = input->measurement;
    me->filtered_derivative = filtered_derivative;
    me->has_previous_sample = true;
    *output = saturated_output;
    return ALG_PID_STATUS_OK;
}

const alg_pid_terms_t *alg_pid_get_terms(const alg_pid_t *me)
{
    return ((me != NULL) && me->is_initialized) ? &me->terms : NULL;
}
