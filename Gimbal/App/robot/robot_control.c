/**
 * @file robot_control.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 机器人控制层初始化与各子模块更新函数
 * @version 1.0
 * @date 2026-08-11
 * @copyright Copyright (c) 2026
 *
 * @details 本文件实现控制层各应用模块（安全、视觉、指令、云台、IMU、
 *          射击、底盘）的初始化，并提供各子系统周期性更新接口。
 *          更新函数内部调用对应 app 模块的更新，并刷新电机 CAN 总线。
 */

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

/**
 * @brief 控制层初始化
 *
 * @param me  机器人实例指针
 * @return bsp_status_t  BSP_STATUS_OK 成功，否则失败
 *
 * @note 依次初始化：安全模块、内部数据交换、视觉（若有 USB VCP）、
 *       指令模块、云台（云台板）、IMU（云台板）、射击应用（若已初始化）、
 *       底盘（底盘板）。
 */
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

    /* 安全监控模块初始化 */
    status = app_safety_init(NULL);
    if (status != BSP_STATUS_OK)
    {
        robot_record_failure(status, "safety", 0);
        return status;
    }

    /* 内部数据交换初始化（用于模块间通信） */
    app_exchange_init();

#ifdef PROJECT_HAS_VISION_USB
    /* 若启用视觉 USB 通信，则初始化视觉模块 */
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

    /* 指令模块初始化（DR16 和板间通信指令解析） */
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
    /* 云台板：初始化云台控制模块 */
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

    /* 云台板：初始化 IMU 姿态解算模块 */
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

    /* 若发射机构已初始化，则初始化射击应用模块 */
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
    /* 底盘板：初始化底盘运动控制模块 */
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

/**
 * @brief 云台子系统的周期性更新
 *
 * @param me            机器人实例指针
 * @param delta_time_s  时间步长（秒）
 *
 * @note 调用 app_gimbal_update 后，刷新 Yaw 电机 CAN 总线
 */
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

/**
 * @brief 底盘子系统的周期性更新
 *
 * @param me            机器人实例指针
 * @param delta_time_s  时间步长（秒）
 *
 * @note 调用 app_chassis_update 后，刷新驱动和转向 CAN 总线
 */
void robot_chassis_update(robot_t *me, float delta_time_s)
{
    if ((me == NULL) || !me->initialized || !me->control.chassis.initialized)
    {
        return;
    }
    app_chassis_update(&me->control.chassis, delta_time_s);
    if ((module_dji_motor_bus_flush(&me->devices.chassis_drive_bus) != MODULE_MOTOR_STATUS_OK) ||
        (module_dji_motor_bus_flush(&me->devices.chassis_steering_bus) != MODULE_MOTOR_STATUS_OK))
    {
        bsp_error_record(BSP_STATUS_IO_ERROR, "chassis_flush", 0);
    }
}

/**
 * @brief 射击子系统的周期性更新
 *
 * @param me            机器人实例指针
 * @param delta_time_s  时间步长（秒）
 *
 * @note 调用 app_shooter_update 后，刷新射击 CAN 总线
 */
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

/**
 * @brief 命令与通信整体更新（主循环调用）
 *
 * @param me              机器人实例指针
 * @param elapsed_time_ms 距上次调用的毫秒数
 *
 * @note 本函数依次调用：
 *       - robot_communication_update 处理底层通信
 *       - app_command_update 更新指令处理
 *       - app_vision_update（若有视觉）
 *       - 统计更新次数
 */
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