/**
 * @file alg_pid.h
 * @brief 面向车载控制的精简 PID 控制器
 * @note C11、静态内存、显式时间步长，不依赖 HAL 或 RTOS。
 */
#ifndef ALG_PID_H
#define ALG_PID_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ALG_PID_STATUS_OK = 0,
    ALG_PID_STATUS_INVALID_ARGUMENT,
    ALG_PID_STATUS_OUT_OF_RANGE,
    ALG_PID_STATUS_NOT_INITIALIZED,
    ALG_PID_STATUS_NUMERICAL_ERROR
} alg_pid_status_t;

typedef struct
{
    float proportional_gain;
    float integral_gain;
    float derivative_gain;
    float derivative_filter_cutoff_hz;
    float integral_min;
    float integral_max;
    float output_min;
    float output_max;
    bool derivative_on_measurement;
} alg_pid_config_t;

typedef struct
{
    float setpoint;
    float measurement;
    float velocity_feedforward;
    float acceleration_feedforward;
    float additional_feedforward;
    float delta_time_s;
} alg_pid_input_t;

typedef struct
{
    float proportional;
    float integral;
    float derivative;
    float feedforward;
    float unsaturated_output;
    float output;
} alg_pid_terms_t;

typedef struct
{
    alg_pid_config_t config;
    alg_pid_terms_t terms;
    float previous_error;
    float previous_measurement;
    float filtered_derivative;
    bool has_previous_sample;
    bool is_initialized;
} alg_pid_t;

alg_pid_status_t alg_pid_config_init(alg_pid_config_t *config);
alg_pid_status_t alg_pid_init(alg_pid_t *me, const alg_pid_config_t *config);
alg_pid_status_t alg_pid_reset(alg_pid_t *me, float measurement, float initial_output);
alg_pid_status_t alg_pid_update(alg_pid_t *me, float setpoint, float measurement,
                                float delta_time_s, float *output);
alg_pid_status_t alg_pid_update_advanced(alg_pid_t *me, const alg_pid_input_t *input,
                                         float *output);
const alg_pid_terms_t *alg_pid_get_terms(const alg_pid_t *me);

typedef struct
{
    alg_pid_config_t position_config;
    alg_pid_config_t velocity_config;
    uint32_t position_loop_divider;
    float velocity_setpoint_min;
    float velocity_setpoint_max;
} alg_pid_cascade_config_t;

typedef struct
{
    float position_setpoint;
    float position_measurement;
    float velocity_measurement;
    float velocity_feedforward;
    float actuator_feedforward;
    float delta_time_s;
} alg_pid_cascade_input_t;

typedef struct
{
    alg_pid_t position_controller;
    alg_pid_t velocity_controller;
    uint32_t position_loop_divider;
    uint32_t position_loop_counter;
    float position_elapsed_time_s;
    float velocity_setpoint_min;
    float velocity_setpoint_max;
    float velocity_setpoint;
    bool is_initialized;
} alg_pid_cascade_t;

alg_pid_status_t alg_pid_cascade_init(alg_pid_cascade_t *me,
                                      const alg_pid_cascade_config_t *config);
alg_pid_status_t alg_pid_cascade_reset(alg_pid_cascade_t *me, float position_measurement,
                                       float velocity_measurement, float initial_output);
alg_pid_status_t alg_pid_cascade_update(alg_pid_cascade_t *me,
                                        const alg_pid_cascade_input_t *input, float *output);
float alg_pid_cascade_get_velocity_setpoint(const alg_pid_cascade_t *me);

typedef struct
{
    alg_pid_cascade_config_t cascade_config;
} alg_pid_angle_config_t;

typedef struct
{
    float target_position_rad;
    float target_velocity_rad_per_s;
    float measured_position_rad;
    float measured_velocity_rad_per_s;
    float actuator_feedforward;
    float delta_time_s;
} alg_pid_angle_input_t;

typedef struct
{
    alg_pid_cascade_t cascade;
} alg_pid_angle_t;

alg_pid_status_t alg_pid_angle_init(alg_pid_angle_t *me, const alg_pid_angle_config_t *config);
alg_pid_status_t alg_pid_angle_reset(alg_pid_angle_t *me, float measured_position_rad,
                                     float measured_velocity_rad_per_s, float initial_output);
alg_pid_status_t alg_pid_angle_update(alg_pid_angle_t *me, const alg_pid_angle_input_t *input,
                                      float *control_output);
float alg_pid_angle_get_velocity_setpoint(const alg_pid_angle_t *me);

#ifdef __cplusplus
}
#endif

#endif
