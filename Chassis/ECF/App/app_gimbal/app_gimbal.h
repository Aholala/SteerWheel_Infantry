#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include "bsp_common.h"
#include "module_board_comm.h"
#include "module_motor.h"

#include <stdbool.h>

typedef struct
{
    module_motor_t *pitch_motor;
    module_motor_t *yaw_motor;
    module_board_comm_t *board_comm;
    float target_tolerance_rad;
} app_gimbal_config_t;

typedef struct
{
    app_gimbal_config_t config;
    bool initialized;
} app_gimbal_t;

bsp_status_t app_gimbal_init(app_gimbal_t *me, const app_gimbal_config_t *config);
void app_gimbal_update(app_gimbal_t *me, float delta_time_s);

#endif
