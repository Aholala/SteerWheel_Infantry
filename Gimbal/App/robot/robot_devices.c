/**
 * @file robot_devices.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 机器人底层设备初始化、通信路由与更新管理
 * @version 1.0
 * @date 2026-08-11
 * @copyright Copyright (c) 2026
 *
 * @details 本文件实现机器人所有硬件外设（IMU、电机、CAN总线、DR16遥控器、
 *          板间通信）的初始化、配置、CAN帧路由以及周期性的通信更新任务。
 *          根据编译宏 APP_BOARD_ROLE 区分云台板/底盘板，条件编译相应模块。
 */

#include "robot_internal.h"
#include "board_config.h"
#include "project_config.h"

/**
 * @brief 构造 PID 配置结构体（带通用滤波器与输出限幅）
 *
 * @param proportional_gain   比例系数
 * @param integral_gain       积分系数
 * @param derivative_gain     微分系数
 * @param output_min          输出下限
 * @param output_max          输出上限
 * @return alg_pid_config_t   填充后的 PID 配置
 */
static alg_pid_config_t robot_pid_config(float proportional_gain, float integral_gain,
                                         float derivative_gain, float output_min, float output_max)
{
    return (alg_pid_config_t){
        .proportional_gain = proportional_gain,
        .integral_gain = integral_gain,
        .derivative_gain = derivative_gain,
        .derivative_filter_cutoff_hz = PROJECT_PID_DERIVATIVE_FILTER_HZ,
        .integral_min = -PROJECT_PID_INTEGRAL_LIMIT,
        .integral_max = PROJECT_PID_INTEGRAL_LIMIT,
        .output_min = output_min,
        .output_max = output_max,
        .derivative_on_measurement = true,
    };
}

#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
/**
 * @brief BMI088 片选控制回调（仅云台板使用）
 *
 * @param user_context  用户上下文（未使用）
 * @param sensor        选择加速度计或陀螺仪
 * @param is_selected   true 选中，false 释放
 */
static void robot_bmi088_chip_select(void *user_context, module_bmi088_sensor_t sensor,
                                     bool is_selected)
{
    (void)user_context;
    board_config_set_bmi088_chip_select(sensor == MODULE_BMI088_SENSOR_GYRO, is_selected);
}

/**
 * @brief BMI088 毫秒延时回调
 *
 * @param user_context  用户上下文（未使用）
 * @param delay_ms      延时毫秒数
 */
static void robot_bmi088_delay_ms(void *user_context, uint32_t delay_ms)
{
    (void)user_context;
    board_config_delay_ms(delay_ms);
}

/**
 * @brief BMI088 微秒时间戳获取回调
 *
 * @param user_context  用户上下文（未使用）
 * @return uint32_t     当前微秒时间
 */
static uint32_t robot_bmi088_get_time_us(void *user_context)
{
    (void)user_context;
    return board_config_get_time_us();
}
#endif

/**
 * @brief 初始化 IMU 设备（BMI088）
 *
 * @param me  机器人实例指针
 * @return bsp_status_t  BSP_STATUS_OK 成功，否则失败
 */
static bsp_status_t robot_imu_device_init(robot_t *me)
{
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    const module_bmi088_config_t imu_config = {
        .logical_name = "bmi088",
        .registration_key = 3U,
        .spi = board_config_get_bmi088_spi(),
        .set_chip_select = robot_bmi088_chip_select,
        .delay_ms = robot_bmi088_delay_ms,
        .get_time_us = robot_bmi088_get_time_us,
        .user_context = NULL,
        .acceleration_range = MODULE_BMI088_ACCEL_RANGE_6G,
        .angular_velocity_range = MODULE_BMI088_GYRO_RANGE_2000_DPS,
        .axis_map =
            {
                {.source_axis = 0U, .direction_sign = 1.0F},
                {.source_axis = 1U, .direction_sign = 1.0F},
                {.source_axis = 2U, .direction_sign = 1.0F},
            },
        .transfer_timeout_ms = 2U,
    };
    const module_bmi088_status_t imu_status = module_bmi088_init(&me->devices.bmi088, &imu_config);
    if (imu_status != MODULE_BMI088_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "bmi088", (int32_t)imu_status);
        return BSP_STATUS_IO_ERROR;
    }
    me->devices.bmi088_initialized = true;
#else
    (void)me;
#endif
    return BSP_STATUS_OK;
}

/**
 * @brief 初始化云台电机（Pitch DM4310 + Yaw GM6020）
 *
 * @param me  机器人实例指针
 * @return bsp_status_t  BSP_STATUS_OK 成功，否则失败
 */
static bsp_status_t robot_gimbal_motors_init(robot_t *me)
{
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    module_motor_status_t motor_status;
    const module_dm4310_config_t pitch_config = {
        .motor_name = "pitch_dm4310",
        .registration_key = 1U,
        .can = me->devices.can[PROJECT_CAN_GIMBAL_PITCH],
        .control_mode = MODULE_DM4310_CONTROL_POSITION_VELOCITY,
        .base_command_identifier = PROJECT_PITCH_DM4310_COMMAND_ID,
        .feedback_identifier = PROJECT_PITCH_DM4310_FEEDBACK_ID,
        .transmit_timeout_ms = PROJECT_BOARD_COMM_TX_TIMEOUT_MS,
        .protocol_limits =
            {
                .position_min_rad = -12.5F,
                .position_max_rad = 12.5F,
                .velocity_min_rad_per_s = -30.0F,
                .velocity_max_rad_per_s = 30.0F,
                .torque_min_nm = -10.0F,
                .torque_max_nm = 10.0F,
                .proportional_gain_min = 0.0F,
                .proportional_gain_max = 500.0F,
                .derivative_gain_min = 0.0F,
                .derivative_gain_max = 5.0F,
            },
    };
    const module_gm6020_config_t yaw_config = {
        .motor_name = "yaw_gm6020",
        .registration_key = 2U,
        .motor_bus = &me->devices.gimbal_yaw_bus,
        .control_mode = MODULE_GM6020_CONTROL_ANGLE,
        .motor_identifier = PROJECT_YAW_GM6020_ID,
        .direction_sign = PROJECT_MOTOR_DIRECTION_FORWARD,
        .maximum_temperature_c = PROJECT_DJI_MAXIMUM_TEMPERATURE_C,
        .current_scale_a_per_count = PROJECT_DJI_CURRENT_SCALE_A_PER_COUNT,
        .position_reference = MODULE_DJI_POSITION_ENCODER_ABSOLUTE,
        .encoder_zero_count = PROJECT_YAW_GM6020_ENCODER_ZERO,
        .position_offset_rad = 0.0F,
        .current_pid_config = robot_pid_config(1.0F, 0.0F, 0.0F, -25000.0F, 25000.0F),
        .velocity_pid_config = robot_pid_config(
            PROJECT_YAW_VELOCITY_KP, PROJECT_YAW_VELOCITY_KI, PROJECT_YAW_VELOCITY_KD,
            -PROJECT_PID_CURRENT_OUTPUT_LIMIT_A, PROJECT_PID_CURRENT_OUTPUT_LIMIT_A),
        .angle_pid_config = robot_pid_config(
            PROJECT_YAW_POSITION_KP, PROJECT_YAW_POSITION_KI, PROJECT_YAW_POSITION_KD,
            -PROJECT_POSITION_VELOCITY_LIMIT_RAD_PER_S, PROJECT_POSITION_VELOCITY_LIMIT_RAD_PER_S),
    };

    /* 初始化云台 Yaw 电机总线（DJI 电机总线） */
    motor_status = module_dji_motor_bus_init(&me->devices.gimbal_yaw_bus,
                                             me->devices.can[PROJECT_CAN_GIMBAL_YAW],
                                             PROJECT_BOARD_COMM_TX_TIMEOUT_MS);
    if (motor_status != MODULE_MOTOR_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "yaw_bus", (int32_t)motor_status);
        return BSP_STATUS_IO_ERROR;
    }

    /* 初始化 Pitch 电机（DM4310）并注册到电机注册表 */
    motor_status = module_dm4310_init(&me->devices.pitch_motor, &pitch_config);
    if (motor_status == MODULE_MOTOR_STATUS_OK)
    {
        motor_status =
            module_dm4310_register(&me->devices.pitch_motor, &me->devices.motor_registry);
    }
    if (motor_status != MODULE_MOTOR_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "pitch_motor", (int32_t)motor_status);
        return BSP_STATUS_IO_ERROR;
    }
    me->devices.pitch_motor_initialized = true;

    /* 初始化 Yaw 电机（GM6020）并注册 */
    motor_status = module_gm6020_init(&me->devices.yaw_motor, &yaw_config);
    if (motor_status == MODULE_MOTOR_STATUS_OK)
    {
        motor_status = module_gm6020_register(&me->devices.yaw_motor, &me->devices.motor_registry);
    }
    if (motor_status != MODULE_MOTOR_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "yaw_motor", (int32_t)motor_status);
        return BSP_STATUS_IO_ERROR;
    }
    me->devices.yaw_motor_initialized = true;

    /* 设置反馈超时 */
    (void)module_motor_set_feedback_timeout(module_dm4310_as_motor(&me->devices.pitch_motor),
                                            PROJECT_MOTOR_FEEDBACK_TIMEOUT_MS);
    (void)module_motor_set_feedback_timeout(module_gm6020_as_motor(&me->devices.yaw_motor),
                                            PROJECT_MOTOR_FEEDBACK_TIMEOUT_MS);
#else
    (void)me;
#endif
    return BSP_STATUS_OK;
}

/**
 * @brief 初始化底盘电机（4 个 M3508 驱动 + 4 个 GM6020 转向）
 *
 * @param me  机器人实例指针
 * @return bsp_status_t  BSP_STATUS_OK 成功，否则失败
 */
static bsp_status_t robot_chassis_motors_init(robot_t *me)
{
#if APP_BOARD_ROLE == APP_BOARD_ROLE_CHASSIS
    static const char *const drive_names[PROJECT_CHASSIS_MODULE_COUNT] = {
        "drive_front_left",
        "drive_front_right",
        "drive_rear_left",
        "drive_rear_right",
    };
    static const char *const steering_names[PROJECT_CHASSIS_MODULE_COUNT] = {
        "steering_front_left",
        "steering_front_right",
        "steering_rear_left",
        "steering_rear_right",
    };
    static const uint8_t drive_identifiers[PROJECT_CHASSIS_MODULE_COUNT] = {
        PROJECT_DRIVE_FRONT_LEFT_ID,
        PROJECT_DRIVE_FRONT_RIGHT_ID,
        PROJECT_DRIVE_REAR_LEFT_ID,
        PROJECT_DRIVE_REAR_RIGHT_ID,
    };
    static const uint8_t steering_identifiers[PROJECT_CHASSIS_MODULE_COUNT] = {
        PROJECT_STEERING_FRONT_LEFT_ID,
        PROJECT_STEERING_FRONT_RIGHT_ID,
        PROJECT_STEERING_REAR_LEFT_ID,
        PROJECT_STEERING_REAR_RIGHT_ID,
    };
    module_motor_status_t motor_status;
    size_t index;

    /* 初始化驱动总线与转向总线 */
    motor_status = module_dji_motor_bus_init(&me->devices.chassis_drive_bus,
                                             me->devices.can[PROJECT_CAN_CHASSIS_DRIVE],
                                             PROJECT_BOARD_COMM_TX_TIMEOUT_MS);
    if (motor_status == MODULE_MOTOR_STATUS_OK)
    {
        motor_status = module_dji_motor_bus_init(&me->devices.chassis_steering_bus,
                                                 me->devices.can[PROJECT_CAN_CHASSIS_STEERING],
                                                 PROJECT_BOARD_COMM_TX_TIMEOUT_MS);
    }
    if (motor_status != MODULE_MOTOR_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "chassis_bus", (int32_t)motor_status);
        return BSP_STATUS_IO_ERROR;
    }

    /* 遍历四个轮子模块，初始化驱动电机和转向电机，并构造 swerve 模块 */
    for (index = 0U; index < PROJECT_CHASSIS_MODULE_COUNT; ++index)
    {
        const module_m3508_config_t drive_config = {
            .motor_name = drive_names[index],
            .registration_key = (uint32_t)(10U + index),
            .motor_bus = &me->devices.chassis_drive_bus,
            .control_mode = MODULE_M3508_CONTROL_VELOCITY,
            .motor_identifier = drive_identifiers[index],
            .direction_sign = PROJECT_MOTOR_DIRECTION_FORWARD,
            .maximum_temperature_c = PROJECT_DJI_MAXIMUM_TEMPERATURE_C,
            .current_scale_a_per_count = PROJECT_DJI_CURRENT_SCALE_A_PER_COUNT,
            .position_reference = MODULE_DJI_POSITION_BOOT_RELATIVE,
            .encoder_zero_count = 0U,
            .position_offset_rad = 0.0F,
            .current_pid_config = robot_pid_config(1.0F, 0.0F, 0.0F, -16384.0F, 16384.0F),
            .velocity_pid_config = robot_pid_config(
                PROJECT_DRIVE_VELOCITY_KP, PROJECT_DRIVE_VELOCITY_KI, PROJECT_DRIVE_VELOCITY_KD,
                -PROJECT_DRIVE_VELOCITY_OUTPUT_LIMIT_A, PROJECT_DRIVE_VELOCITY_OUTPUT_LIMIT_A),
        };
        const module_gm6020_config_t steering_config = {
            .motor_name = steering_names[index],
            .registration_key = (uint32_t)(20U + index),
            .motor_bus = &me->devices.chassis_steering_bus,
            .control_mode = MODULE_GM6020_CONTROL_ANGLE,
            .motor_identifier = steering_identifiers[index],
            .direction_sign = PROJECT_MOTOR_DIRECTION_FORWARD,
            .maximum_temperature_c = PROJECT_DJI_MAXIMUM_TEMPERATURE_C,
            .current_scale_a_per_count = PROJECT_DJI_CURRENT_SCALE_A_PER_COUNT,
            .position_reference = MODULE_DJI_POSITION_ENCODER_ABSOLUTE,
            .encoder_zero_count = 0U,
            .position_offset_rad = 0.0F,
            .current_pid_config = robot_pid_config(1.0F, 0.0F, 0.0F, -25000.0F, 25000.0F),
            .velocity_pid_config =
                robot_pid_config(PROJECT_STEERING_VELOCITY_KP, PROJECT_STEERING_VELOCITY_KI,
                                 PROJECT_STEERING_VELOCITY_KD, -PROJECT_PID_CURRENT_OUTPUT_LIMIT_A,
                                 PROJECT_PID_CURRENT_OUTPUT_LIMIT_A),
            .angle_pid_config = robot_pid_config(
                PROJECT_STEERING_POSITION_KP, PROJECT_STEERING_POSITION_KI,
                PROJECT_STEERING_POSITION_KD, -PROJECT_POSITION_VELOCITY_LIMIT_RAD_PER_S,
                PROJECT_POSITION_VELOCITY_LIMIT_RAD_PER_S),
        };
        module_swerve_config_t swerve_config;

        motor_status = module_m3508_init(&me->devices.drive_motors[index], &drive_config);
        if (motor_status == MODULE_MOTOR_STATUS_OK)
        {
            motor_status = module_m3508_register(&me->devices.drive_motors[index],
                                                 &me->devices.motor_registry);
        }
        if (motor_status == MODULE_MOTOR_STATUS_OK)
        {
            motor_status =
                module_gm6020_init(&me->devices.steering_motors[index], &steering_config);
        }
        if (motor_status == MODULE_MOTOR_STATUS_OK)
        {
            motor_status = module_gm6020_register(&me->devices.steering_motors[index],
                                                  &me->devices.motor_registry);
        }
        if (motor_status != MODULE_MOTOR_STATUS_OK)
        {
            robot_record_failure(BSP_STATUS_IO_ERROR, "chassis_motor", (int32_t)motor_status);
            return BSP_STATUS_IO_ERROR;
        }

        /* 配置 Swerve 模块（将驱动电机和转向电机组合） */
        swerve_config = (module_swerve_config_t){
            .drive_motor = module_m3508_as_motor(&me->devices.drive_motors[index]),
            .steering_motor = module_gm6020_as_motor(&me->devices.steering_motors[index]),
            .wheel_radius_m = PROJECT_CHASSIS_WHEEL_RADIUS_M,
            .drive_reduction_ratio = PROJECT_CHASSIS_DRIVE_REDUCTION_RATIO,
            .steering_zero_offset_rad = 0.0F,
            .drive_direction_sign = PROJECT_MOTOR_DIRECTION_FORWARD,
            .steering_direction_sign = PROJECT_MOTOR_DIRECTION_FORWARD,
        };
        if (module_swerve_init(&me->devices.swerve_modules[index], &swerve_config) !=
            MODULE_SWERVE_STATUS_OK)
        {
            robot_record_failure(BSP_STATUS_INVALID_ARGUMENT, "swerve", (int32_t)index);
            return BSP_STATUS_INVALID_ARGUMENT;
        }
    }

    /* 配置底盘运动学（矩形布局）并初始化 */
    if ((alg_swerve_configure_rectangular_layout(
             me->devices.chassis_geometry, PROJECT_CHASSIS_HALF_WHEELBASE_M,
             PROJECT_CHASSIS_HALF_TRACK_M) != ALG_SWERVE_STATUS_OK) ||
        (alg_swerve_init(&me->devices.chassis_kinematics, me->devices.chassis_geometry,
                         PROJECT_CHASSIS_MODULE_COUNT,
                         PROJECT_CHASSIS_MAX_WHEEL_VELOCITY_M_PER_S) != ALG_SWERVE_STATUS_OK))
    {
        robot_record_failure(BSP_STATUS_INVALID_ARGUMENT, "chassis_kinematics", 0);
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    me->devices.chassis_initialized = true;
#else
    (void)me;
#endif
    return BSP_STATUS_OK;
}

/**
 * @brief 初始化发射机构（摩擦轮 + 拨弹电机）
 *
 * @param me  机器人实例指针
 * @return bsp_status_t  BSP_STATUS_OK 成功，否则失败
 */
static bsp_status_t robot_shooter_device_init(robot_t *me)
{
#if (APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL) ||                                                   \
    ((APP_BOARD_ROLE == APP_BOARD_ROLE_CHASSIS) &&                                                 \
     (PROJECT_FEEDER_LOCATION == APP_DEVICE_LOCATION_CHASSIS))
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    static const char *const friction_names[2] = {"friction_left", "friction_right"};
    static const uint8_t friction_identifiers[2] = {PROJECT_FRICTION_LEFT_ID,
                                                    PROJECT_FRICTION_RIGHT_ID};
#endif
    module_motor_status_t motor_status;
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    size_t index;
#endif

    /* 初始化发射机构 CAN 总线 */
    motor_status =
        module_dji_motor_bus_init(&me->devices.shooter_bus, me->devices.can[PROJECT_CAN_SHOOTER],
                                  PROJECT_BOARD_COMM_TX_TIMEOUT_MS);
    if (motor_status != MODULE_MOTOR_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "shooter_bus", (int32_t)motor_status);
        return BSP_STATUS_IO_ERROR;
    }

#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    /* 云台板：初始化左右摩擦轮（M3508） */
    for (index = 0U; index < 2U; ++index)
    {
        const module_m3508_config_t friction_config = {
            .motor_name = friction_names[index],
            .registration_key = (uint32_t)(30U + index),
            .motor_bus = &me->devices.shooter_bus,
            .control_mode = MODULE_M3508_CONTROL_VELOCITY,
            .motor_identifier = friction_identifiers[index],
            .direction_sign = PROJECT_MOTOR_DIRECTION_FORWARD,
            .maximum_temperature_c = PROJECT_DJI_MAXIMUM_TEMPERATURE_C,
            .current_scale_a_per_count = PROJECT_DJI_CURRENT_SCALE_A_PER_COUNT,
            .position_reference = MODULE_DJI_POSITION_BOOT_RELATIVE,
            .current_pid_config = robot_pid_config(1.0F, 0.0F, 0.0F, -16384.0F, 16384.0F),
            .velocity_pid_config = robot_pid_config(
                PROJECT_DRIVE_VELOCITY_KP, PROJECT_DRIVE_VELOCITY_KI, PROJECT_DRIVE_VELOCITY_KD,
                -PROJECT_DRIVE_VELOCITY_OUTPUT_LIMIT_A, PROJECT_DRIVE_VELOCITY_OUTPUT_LIMIT_A),
        };
        motor_status = module_m3508_init(&me->devices.friction_motors[index], &friction_config);
        if (motor_status == MODULE_MOTOR_STATUS_OK)
        {
            motor_status = module_m3508_register(&me->devices.friction_motors[index],
                                                 &me->devices.motor_registry);
        }
        if (motor_status != MODULE_MOTOR_STATUS_OK)
        {
            robot_record_failure(BSP_STATUS_IO_ERROR, "friction_motor", (int32_t)motor_status);
            return BSP_STATUS_IO_ERROR;
        }
    }
#endif

    /* 若拨弹电机与当前板同体，则初始化拨弹电机（M3508 角度控制） */
#if ((APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL) &&                                                  \
     (PROJECT_FEEDER_LOCATION == APP_DEVICE_LOCATION_GIMBAL)) ||                                   \
    ((APP_BOARD_ROLE == APP_BOARD_ROLE_CHASSIS) &&                                                 \
     (PROJECT_FEEDER_LOCATION == APP_DEVICE_LOCATION_CHASSIS))
    {
        const module_m3508_config_t feeder_config = {
            .motor_name = "feeder",
            .registration_key = 32U,
            .motor_bus = &me->devices.shooter_bus,
            .control_mode = MODULE_M3508_CONTROL_ANGLE,
            .motor_identifier = PROJECT_FEEDER_ID,
            .direction_sign = PROJECT_MOTOR_DIRECTION_FORWARD,
            .maximum_temperature_c = PROJECT_DJI_MAXIMUM_TEMPERATURE_C,
            .current_scale_a_per_count = PROJECT_DJI_CURRENT_SCALE_A_PER_COUNT,
            .position_reference = MODULE_DJI_POSITION_BOOT_RELATIVE,
            .current_pid_config = robot_pid_config(1.0F, 0.0F, 0.0F, -16384.0F, 16384.0F),
            .velocity_pid_config =
                robot_pid_config(PROJECT_STEERING_VELOCITY_KP, PROJECT_STEERING_VELOCITY_KI,
                                 PROJECT_STEERING_VELOCITY_KD, -PROJECT_PID_CURRENT_OUTPUT_LIMIT_A,
                                 PROJECT_PID_CURRENT_OUTPUT_LIMIT_A),
            .angle_pid_config = robot_pid_config(
                PROJECT_STEERING_POSITION_KP, PROJECT_STEERING_POSITION_KI,
                PROJECT_STEERING_POSITION_KD, -PROJECT_POSITION_VELOCITY_LIMIT_RAD_PER_S,
                PROJECT_POSITION_VELOCITY_LIMIT_RAD_PER_S),
        };
        motor_status = module_m3508_init(&me->devices.feeder_motor, &feeder_config);
        if (motor_status == MODULE_MOTOR_STATUS_OK)
        {
            motor_status =
                module_m3508_register(&me->devices.feeder_motor, &me->devices.motor_registry);
        }
        if (motor_status != MODULE_MOTOR_STATUS_OK)
        {
            robot_record_failure(BSP_STATUS_IO_ERROR, "feeder_motor", (int32_t)motor_status);
            return BSP_STATUS_IO_ERROR;
        }
    }
#endif

    /* 初始化射击控制模块（封装摩擦轮 + 拨弹逻辑） */
    {
        const module_shooter_config_t shooter_config = {
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
            .left_friction_motor = module_m3508_as_motor(&me->devices.friction_motors[0]),
            .right_friction_motor = module_m3508_as_motor(&me->devices.friction_motors[1]),
#endif
#if ((APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL) &&                                                  \
     (PROJECT_FEEDER_LOCATION == APP_DEVICE_LOCATION_GIMBAL)) ||                                   \
    ((APP_BOARD_ROLE == APP_BOARD_ROLE_CHASSIS) &&                                                 \
     (PROJECT_FEEDER_LOCATION == APP_DEVICE_LOCATION_CHASSIS))
            .feeder_motor = module_m3508_as_motor(&me->devices.feeder_motor),
#endif
            .left_friction_direction_sign = 1.0F,
            .right_friction_direction_sign = -1.0F,
            .feeder_direction_sign = 1.0F,
            .feeder_step_rad = 0.7853982F,
            .feeder_position_tolerance_rad = 0.02F,
            .jam_velocity_threshold_rad_per_s = 0.2F,
            .jam_current_threshold_a = 5.0F,
            .jam_current_threshold_raw = 8000,
            .jam_confirmation_time_s = 0.15F,
            .rollback_angle_rad = 0.35F,
            .rollback_position_tolerance_rad = 0.03F,
            .rollback_timeout_s = 0.5F,
            .friction_velocity_tolerance_rad_per_s = 20.0F,
            .friction_ready_time_s = 0.2F,
            .fire_stable_time_s = 0.1F,
            .automatic_shot_interval_s = 0.15F,
            .maximum_pending_shots = 3U,
            .maximum_jam_retries = 3U,
        };
        if (module_shooter_init(&me->devices.shooter, &shooter_config) != MODULE_SHOOTER_STATUS_OK)
        {
            robot_record_failure(BSP_STATUS_INVALID_ARGUMENT, "shooter", 0);
            return BSP_STATUS_INVALID_ARGUMENT;
        }
    }
    me->devices.shooter_initialized = true;
#else
    (void)me;
#endif
    return BSP_STATUS_OK;
}

/**
 * @brief 判断 DR16 遥控器是否挂载在当前板上
 *
 * @return true  本地 DR16
 * @return false 远程 DR16（通过板间通信获取）
 */
static bool robot_local_dr16_selected(void)
{
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    return APP_DR16_LOCATION == APP_DEVICE_LOCATION_GIMBAL;
#else
    return APP_DR16_LOCATION == APP_DEVICE_LOCATION_CHASSIS;
#endif
}

/**
 * @brief CAN 帧路由函数，根据 CAN 索引分发到对应的设备模块
 *
 * @param me         机器人实例指针
 * @param can_index  CAN 总线索引
 * @param frame      接收到的 CAN 帧指针
 */
static void robot_route_can_frame(robot_t *me, size_t can_index, const bsp_can_frame_t *frame)
{
    if ((me == NULL) || (frame == NULL))
    {
        return;
    }
    /* 板间通信帧 */
    if (can_index == (size_t)PROJECT_CAN_BOARD_LINK)
    {
        (void)module_board_comm_handle_frame(&me->devices.board_comm, frame);
    }
#if APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    /* 云台板：Pitch CAN 接收 DM4310 反馈及射击总线反馈 */
    if (can_index == (size_t)PROJECT_CAN_GIMBAL_PITCH)
    {
        (void)module_dm4310_handle_feedback(&me->devices.pitch_motor, frame);
        (void)module_dji_motor_bus_handle_feedback(&me->devices.shooter_bus, frame);
    }
    /* 云台板：Yaw CAN 接收 GM6020 反馈 */
    if (can_index == (size_t)PROJECT_CAN_GIMBAL_YAW)
    {
        (void)module_dji_motor_bus_handle_feedback(&me->devices.gimbal_yaw_bus, frame);
    }
#endif
#if APP_BOARD_ROLE == APP_BOARD_ROLE_CHASSIS
    /* 底盘板：驱动 CAN 接收驱动电机反馈，若拨弹电机在底盘则也接收射击总线反馈 */
    if (can_index == (size_t)PROJECT_CAN_CHASSIS_DRIVE)
    {
        (void)module_dji_motor_bus_handle_feedback(&me->devices.chassis_drive_bus, frame);
#if PROJECT_FEEDER_LOCATION == APP_DEVICE_LOCATION_CHASSIS
        (void)module_dji_motor_bus_handle_feedback(&me->devices.shooter_bus, frame);
#endif
    }
    /* 底盘板：转向 CAN 接收转向电机反馈 */
    if (can_index == (size_t)PROJECT_CAN_CHASSIS_STEERING)
    {
        (void)module_dji_motor_bus_handle_feedback(&me->devices.chassis_steering_bus, frame);
    }
#endif
}

/**
 * @brief 机器人设备层初始化（外部接口）
 *
 * @param me  机器人实例指针
 * @return bsp_status_t  BSP_STATUS_OK 成功，否则失败
 *
 * @note 本函数依次完成：
 *       - 参数校验与板级配置检查
 *       - 电机注册表初始化
 *       - 3 路 CAN 启动
 *       - 板间通信初始化
 *       - 云台电机、底盘电机、IMU、发射机构初始化
 *       - 本地 DR16 遥控器初始化（若存在）
 */
bsp_status_t robot_devices_init(robot_t *me)
{
    bsp_status_t status;
    int32_t module_error;
    module_board_comm_config_t board_comm_config;
    size_t can_index;

    if (me == NULL)
    {
        robot_record_failure(BSP_STATUS_INVALID_ARGUMENT, "robot_devices", 0);
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (!board_config_is_initialized())
    {
        robot_record_failure(BSP_STATUS_NOT_INITIALIZED, "board_config", 0);
        return BSP_STATUS_NOT_INITIALIZED;
    }

    /* 初始化电机注册表（用于统一管理所有电机） */
    if (module_motor_registry_init(&me->devices.motor_registry, me->devices.motor_storage,
                                   sizeof(me->devices.motor_storage) /
                                       sizeof(me->devices.motor_storage[0])) !=
        MODULE_MOTOR_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_NO_RESOURCE, "motor_registry", 0);
        return BSP_STATUS_NO_RESOURCE;
    }

    /* 启动 3 路 CAN 总线 */
    for (can_index = 0U; can_index < 3U; ++can_index)
    {
        me->devices.can[can_index] = board_config_get_can((board_config_can_index_t)can_index);
        if (me->devices.can[can_index] == NULL)
        {
            robot_record_failure(BSP_STATUS_IO_ERROR, "can", (int32_t)can_index);
            return BSP_STATUS_IO_ERROR;
        }
        status = bsp_can_start(me->devices.can[can_index]);
        if (status != BSP_STATUS_OK)
        {
            robot_record_failure(status, "can_start", (int32_t)can_index);
            return status;
        }
        me->devices.can_started[can_index] = true;
    }

    /* 保存板间通信专用 CAN 句柄 */
    me->devices.link_can = me->devices.can[PROJECT_CAN_BOARD_LINK];
    if (me->devices.link_can == NULL)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "link_can", 0);
        return BSP_STATUS_IO_ERROR;
    }

    /* 初始化板间通信模块 */
    board_comm_config = (module_board_comm_config_t){
        .can = me->devices.link_can,
        .transmit_base_identifier = PROJECT_BOARD_COMM_TX_BASE_ID,
        .receive_base_identifier = PROJECT_BOARD_COMM_RX_BASE_ID,
        .transmit_timeout_ms = PROJECT_BOARD_COMM_TX_TIMEOUT_MS,
        .offline_timeout_ms = PROJECT_BOARD_COMM_OFFLINE_MS,
    };
    module_error = (int32_t)module_board_comm_init(&me->devices.board_comm, &board_comm_config);
    if (module_error != (int32_t)MODULE_BOARD_COMM_STATUS_OK)
    {
        robot_record_failure(BSP_STATUS_IO_ERROR, "board_comm", module_error);
        return BSP_STATUS_IO_ERROR;
    }
    me->devices.board_comm_initialized = true;

    /* 依次初始化各设备组 */
    status = robot_gimbal_motors_init(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = robot_chassis_motors_init(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = robot_imu_device_init(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    status = robot_shooter_device_init(me);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    /* 根据配置决定 DR16 是否本地，并初始化 */
    me->devices.dr16_is_local = robot_local_dr16_selected();
    if (me->devices.dr16_is_local)
    {
        bsp_usart_t *const usart = board_config_get_usart(BOARD_CONFIG_UART_DR16);
        module_dr16_config_t dr16_config;

        if (usart == NULL)
        {
            robot_record_failure(BSP_STATUS_IO_ERROR, "dr16_usart", 0);
            return BSP_STATUS_IO_ERROR;
        }

        dr16_config = (module_dr16_config_t){
            .logical_name = "dr16",
            .registration_key = 1U,
            .usart = usart,
            .dma_receive_buffer = me->devices.dr16_buffers,
            .channel_deadband = 10,
            .offline_timeout_ms = PROJECT_DR16_OFFLINE_TIMEOUT_MS,
            .frame_callback = NULL,
            .user_context = NULL,
        };
        module_error = (int32_t)module_dr16_init(&me->devices.dr16, &dr16_config);
        if (module_error != (int32_t)MODULE_DR16_STATUS_OK)
        {
            robot_record_failure(BSP_STATUS_IO_ERROR, "dr16_init", module_error);
            return BSP_STATUS_IO_ERROR;
        }
        module_error = (int32_t)module_dr16_start(&me->devices.dr16);
        if (module_error != (int32_t)MODULE_DR16_STATUS_OK)
        {
            robot_record_failure(BSP_STATUS_IO_ERROR, "dr16_start", module_error);
            return BSP_STATUS_IO_ERROR;
        }
        me->devices.dr16_started = true;
    }

    return BSP_STATUS_OK;
}

/**
 * @brief 设备层反初始化（释放资源）
 *
 * @param me  机器人实例指针
 */
void robot_devices_deinit(robot_t *me)
{
    if (me == NULL)
    {
        return;
    }
    if (me->devices.dr16_started)
    {
        (void)module_dr16_stop(&me->devices.dr16);
        me->devices.dr16_started = false;
    }
    /* 逆序停止 CAN 总线 */
    for (size_t can_index = 3U; can_index > 0U; --can_index)
    {
        const size_t index = can_index - 1U;
        if (me->devices.can_started[index])
        {
            (void)bsp_can_stop(me->devices.can[index]);
            me->devices.can_started[index] = false;
        }
    }
    if (me->devices.board_comm_initialized)
    {
        (void)module_board_comm_deinit(&me->devices.board_comm);
        me->devices.board_comm_initialized = false;
    }
}

/**
 * @brief 通信更新任务（轮询 CAN 接收、DR16 解析、超时更新）
 *
 * @param me              机器人实例指针
 * @param elapsed_time_ms 距上次调用的毫秒数
 *
 * @note 该函数应在主循环中周期性调用，负责：
 *       - 若本地 DR16，则调用 DR16 处理
 *       - 更新板间通信超时
 *       - 更新所有电机反馈超时
 *       - 遍历所有 CAN 总线的两个 FIFO，接收并路由帧
 *       - 统计更新次数
 */
void robot_communication_update(robot_t *me, uint32_t elapsed_time_ms)
{
    bsp_can_frame_t frame;
    size_t can_index;
    size_t motor_index;

    if ((me == NULL) || !me->initialized)
    {
        return;
    }
    if (me->devices.dr16_is_local)
    {
        (void)module_dr16_process(&me->devices.dr16);
        module_dr16_update_time(&me->devices.dr16, elapsed_time_ms);
    }
    module_board_comm_update_time(&me->devices.board_comm, elapsed_time_ms);
    for (motor_index = 0U; motor_index < me->devices.motor_registry.motor_count; ++motor_index)
    {
        (void)module_motor_update_feedback_time(me->devices.motor_storage[motor_index],
                                                elapsed_time_ms);
    }
    /* 从所有 CAN 总线的 FIFO0 和 FIFO1 读取帧并路由 */
    for (can_index = 0U; can_index < 3U; ++can_index)
    {
        while (bsp_can_receive(me->devices.can[can_index], BSP_CAN_RX_FIFO_0, &frame) ==
               BSP_STATUS_OK)
        {
            robot_route_can_frame(me, can_index, &frame);
        }
        while (bsp_can_receive(me->devices.can[can_index], BSP_CAN_RX_FIFO_1, &frame) ==
               BSP_STATUS_OK)
        {
            robot_route_can_frame(me, can_index, &frame);
        }
    }
    if (robot_observer.communication_update_count != UINT32_MAX)
    {
        ++robot_observer.communication_update_count;
    }
}