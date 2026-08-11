#ifndef ROBOT_H
#define ROBOT_H

#include "bsp_common.h"
#include "bsp_can.h"
#include "app_chassis.h"
#include "app_gimbal.h"
#include "app_imu.h"
#include "app_shooter.h"
#include "module_board_comm.h"
#include "module_bmi088.h"
#include "module_dji_motor.h"
#include "module_dm4310.h"
#include "module_dr16.h"
#include "module_gm6020.h"
#include "module_m3508.h"
#include "module_motor.h"
#include "module_swerve.h"
#include "module_shooter.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ROBOT_INIT_STATE_RESET = 0,
    ROBOT_INIT_STATE_DEVICES,
    ROBOT_INIT_STATE_CONTROL,
    ROBOT_INIT_STATE_READY,
    ROBOT_INIT_STATE_FAILED
} robot_init_state_t;

typedef struct
{
    module_dr16_t dr16;
    uint8_t dr16_buffers[2][MODULE_DR16_DMA_BUFFER_SIZE];
    module_board_comm_t board_comm;
    module_motor_registry_t motor_registry;
    module_motor_t *motor_storage[16];
    module_dji_motor_bus_t gimbal_yaw_bus;
    module_dji_motor_bus_t chassis_drive_bus;
    module_dji_motor_bus_t chassis_steering_bus;
    module_dji_motor_bus_t shooter_bus;
    module_dm4310_t pitch_motor;
    module_gm6020_t yaw_motor;
    module_bmi088_t bmi088;
    module_m3508_t drive_motors[4];
    module_gm6020_t steering_motors[4];
    module_m3508_t friction_motors[2];
    module_m3508_t feeder_motor;
    module_shooter_t shooter;
    module_swerve_t swerve_modules[4];
    alg_swerve_t chassis_kinematics;
    alg_swerve_module_geometry_t chassis_geometry[4];
    bsp_can_t *can[3];
    bsp_can_t *link_can;
    bool dr16_is_local;
    bool board_comm_initialized;
    bool can_started[3];
    bool pitch_motor_initialized;
    bool yaw_motor_initialized;
    bool bmi088_initialized;
    bool chassis_initialized;
    bool shooter_initialized;
    bool dr16_started;
} robot_devices_t;

typedef struct
{
    app_chassis_t chassis;
    app_gimbal_t gimbal;
    app_imu_t imu;
    app_shooter_t shooter;
} robot_control_t;

typedef struct
{
    robot_devices_t devices;
    robot_control_t control;
    bool initialized;
} robot_t;

/**
 * @brief Ozone 观测窗口，保持全局符号和固定地址。
 * @note 只存放运行状态，不参与控制计算，调试器写入不会改变机器人行为。
 */
typedef struct
{
    volatile robot_init_state_t init_state;
    volatile bsp_status_t last_status;
    volatile int32_t module_error;
    volatile uint32_t communication_update_count;
    volatile uint32_t command_update_count;
    volatile bool dr16_is_local;
    volatile bool initialized;
    const char *volatile failed_step;
} robot_observer_t;

/* 非 static，便于 Ozone 通过稳定符号直接观察。 */
extern robot_t robot;
extern robot_observer_t robot_observer;

bsp_status_t robot_init(robot_t *me);
robot_t *robot_get(void);
void robot_communication_update(robot_t *me, uint32_t elapsed_time_ms);
void robot_command_update(robot_t *me, uint32_t elapsed_time_ms);
void robot_gimbal_update(robot_t *me, float delta_time_s);
void robot_chassis_update(robot_t *me, float delta_time_s);
void robot_shooter_update(robot_t *me, float delta_time_s);

#endif
