#include "app_gimbal.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"

#include <math.h>

bsp_status_t app_gimbal_init(app_gimbal_t *me, const app_gimbal_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->pitch_motor == NULL) ||
        (config->yaw_motor == NULL) ||
        (config->target_tolerance_rad < 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    *me = (app_gimbal_t){
        .config = *config,
        .initialized = true,
    };
    return BSP_STATUS_OK;
}

void app_gimbal_update(app_gimbal_t *me, float delta_time_s)
{
    app_gimbal_command_t command;
    app_imu_snapshot_t imu;
    app_gimbal_feedback_t feedback = {0};
    const module_motor_feedback_t *pitch_feedback;
    const module_motor_feedback_t *yaw_feedback;
    float pitch_position;
    float yaw_position;

    if ((me == NULL) || !me->initialized)
    {
        return;
    }
    app_exchange_read_gimbal_command(&command);
    app_exchange_read_imu(&imu);
    pitch_feedback = module_motor_get_feedback(me->config.pitch_motor);
    yaw_feedback = module_motor_get_feedback(me->config.yaw_motor);
    if (!command.enabled || (pitch_feedback == NULL) || (yaw_feedback == NULL))
    {
        (void)module_motor_disable(me->config.pitch_motor);
        (void)module_motor_disable(me->config.yaw_motor);
        app_exchange_publish_gimbal_feedback(&feedback);
        return;
    }

    pitch_position = (command.feedback_mode == APP_GIMBAL_FEEDBACK_IMU) && imu.valid
                         ? imu.pitch_rad
                         : pitch_feedback->position_rad;
    yaw_position = (command.feedback_mode == APP_GIMBAL_FEEDBACK_IMU) && imu.valid
                       ? imu.yaw_rad
                       : yaw_feedback->position_rad;
    (void)module_motor_enable(me->config.pitch_motor);
    (void)module_motor_enable(me->config.yaw_motor);
    /* 云台电机必须配置为角度模式，目标值的单位明确为 rad。 */
    (void)module_motor_set_target(me->config.pitch_motor, command.pitch_target_rad);
    (void)module_motor_set_target(me->config.yaw_motor, command.yaw_target_rad);
    (void)module_motor_update(me->config.pitch_motor, delta_time_s);
    (void)module_motor_update(me->config.yaw_motor, delta_time_s);

    feedback.pitch_rad = pitch_position;
    feedback.yaw_rad = yaw_position;
    feedback.pitch_velocity_rad_per_s = pitch_feedback->velocity_rad_per_s;
    feedback.yaw_velocity_rad_per_s = yaw_feedback->velocity_rad_per_s;
    feedback.motors_online = pitch_feedback->is_online && yaw_feedback->is_online;
    feedback.target_locked =
        (fabsf(command.pitch_target_rad - pitch_position) <=
         me->config.target_tolerance_rad) &&
        (fabsf(command.yaw_target_rad - yaw_position) <= me->config.target_tolerance_rad);
    app_exchange_publish_gimbal_feedback(&feedback);

    if (me->config.board_comm != NULL)
    {
        const module_board_comm_gimbal_process_data_t board_data = {
            .yaw_rad = feedback.yaw_rad,
            .pitch_rad = feedback.pitch_rad,
            .yaw_velocity_rad_per_s = feedback.yaw_velocity_rad_per_s,
            .pitch_velocity_rad_per_s = feedback.pitch_velocity_rad_per_s,
            .imu_valid = imu.valid,
            .motors_online = feedback.motors_online,
        };
        if (module_board_comm_send_gimbal(me->config.board_comm, &board_data) !=
            MODULE_BOARD_COMM_STATUS_OK)
        {
            bsp_error_record(BSP_STATUS_IO_ERROR, "send_gimbal", 0);
        }
    }
}
