#include "robot_internal.h"

#include "app_command.h"
#include "app_chassis.h"
#include "app_exchange.h"
#include "app_gimbal.h"
#include "app_imu.h"
#include "app_safety.h"
#include "app_shooter.h"
#include "app_vision.h"
#include "board_config.h"
#include "project_config.h"

bsp_status_t robot_control_init(robot_t *me)
{
    bsp_status_t status;
    app_command_config_t command_config;
    app_vision_config_t vision_config;
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    app_gimbal_config_t gimbal_config;
    app_imu_config_t imu_config;
#endif
    app_shooter_config_t shooter_config;
#if APP_BOARD_ROLE == APP_BOARD_ROLE_CHASSIS
    app_chassis_config_t chassis_config;
    size_t chassis_index;
#endif

    if (me == NULL)
    {
        robot_record_failure(BSP_STATUS_INVALID_ARGUMENT, "control", 0);
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    status = app_safety_init(NULL);
    if (status != BSP_STATUS_OK)
    {
        robot_record_failure(status, "safety", 0);
        return status;
    }

    app_exchange_init();
#ifdef PROJECT_HAS_VISION_USB
    vision_config = (app_vision_config_t){
        .usb_vcp = board_config_get_usb_vcp(),
        .target_timeout_ms = 200U,
        .transmit_period_ms = 10U,
    };
    status = app_vision_init(&vision_config);
    if (status != BSP_STATUS_OK)
    {
        robot_record_failure(status, "vision", 0);
        return status;
    }
#else
    (void)vision_config;
#endif

    command_config = (app_command_config_t){
        .dr16 = me->devices.dr16_is_local ? &me->devices.dr16 : NULL,
        .board_comm = &me->devices.board_comm,
        .dr16_is_local = me->devices.dr16_is_local,
    };
    status = app_command_init(&command_config);
    if (status != BSP_STATUS_OK)
    {
        robot_record_failure(status, "command", 0);
        return status;
    }

#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    gimbal_config = (app_gimbal_config_t){
        .pitch_motor = module_dm4310_as_motor(&me->devices.pitch_motor),
        .yaw_motor = module_gm6020_as_motor(&me->devices.yaw_motor),
        .board_comm = &me->devices.board_comm,
        .target_tolerance_rad = PROJECT_GIMBAL_TARGET_TOLERANCE_RAD,
    };
    status = app_gimbal_init(&me->control.gimbal, &gimbal_config);
    if (status != BSP_STATUS_OK)
    {
        robot_record_failure(status, "gimbal", 0);
        return status;
    }
    imu_config = (app_imu_config_t){
        .sensor = &me->devices.bmi088,
        .accelerometer_correction_gain = 0.01F,
    };
    status = app_imu_init(&me->control.imu, &imu_config);
    if (status != BSP_STATUS_OK)
    {
        robot_record_failure(status, "imu", 0);
        return status;
    }
#endif
    if (me->devices.shooter_initialized)
    {
        shooter_config = (app_shooter_config_t){
            .shooter = &me->devices.shooter,
            .board_comm = &me->devices.board_comm,
        };
        status = app_shooter_init(&me->control.shooter, &shooter_config);
        if (status != BSP_STATUS_OK)
        {
            robot_record_failure(status, "shooter_app", 0);
            return status;
        }
    }

#if APP_BOARD_ROLE == APP_BOARD_ROLE_CHASSIS
    chassis_config = (app_chassis_config_t){
        .kinematics = &me->devices.chassis_kinematics,
        .board_comm = &me->devices.board_comm,
    };
    for (chassis_index = 0U; chassis_index < PROJECT_CHASSIS_MODULE_COUNT; ++chassis_index)
    {
        chassis_config.modules[chassis_index] = &me->devices.swerve_modules[chassis_index];
    }
    status = app_chassis_init(&me->control.chassis, &chassis_config);
    if (status != BSP_STATUS_OK)
    {
        robot_record_failure(status, "chassis", 0);
        return status;
    }
#endif

    return BSP_STATUS_OK;
}

void robot_gimbal_update(robot_t *me, float delta_time_s)
{
    if ((me == NULL) || !me->initialized || !me->control.gimbal.initialized)
    {
        return;
    }
    app_gimbal_update(&me->control.gimbal, delta_time_s);
    if (module_dji_motor_bus_flush(&me->devices.gimbal_yaw_bus) != MODULE_MOTOR_STATUS_OK)
    {
        bsp_error_record(BSP_STATUS_IO_ERROR, "yaw_flush", 0);
    }
}

void robot_chassis_update(robot_t *me, float delta_time_s)
{
    if ((me == NULL) || !me->initialized || !me->control.chassis.initialized)
    {
        return;
    }
    app_chassis_update(&me->control.chassis, delta_time_s);
    if ((module_dji_motor_bus_flush(&me->devices.chassis_drive_bus) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_dji_motor_bus_flush(&me->devices.chassis_steering_bus) !=
         MODULE_MOTOR_STATUS_OK))
    {
        bsp_error_record(BSP_STATUS_IO_ERROR, "chassis_flush", 0);
    }
}

void robot_shooter_update(robot_t *me, float delta_time_s)
{
    if ((me == NULL) || !me->initialized || !me->control.shooter.initialized)
    {
        return;
    }
    app_shooter_update(&me->control.shooter, delta_time_s);
    if (module_dji_motor_bus_flush(&me->devices.shooter_bus) != MODULE_MOTOR_STATUS_OK)
    {
        bsp_error_record(BSP_STATUS_IO_ERROR, "shooter_flush", 0);
    }
}

void robot_command_update(robot_t *me, uint32_t elapsed_time_ms)
{
    if ((me == NULL) || !me->initialized)
    {
        return;
    }

    robot_communication_update(me, elapsed_time_ms);
    app_command_update((float)elapsed_time_ms * 0.001F);
#ifdef PROJECT_HAS_VISION_USB
    app_vision_update(elapsed_time_ms);
#endif
    if (robot_observer.command_update_count != UINT32_MAX)
    {
        ++robot_observer.command_update_count;
    }
}
