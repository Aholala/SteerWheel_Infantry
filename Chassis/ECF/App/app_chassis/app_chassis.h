#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "alg_swerve.h"
#include "bsp_common.h"
#include "module_board_comm.h"
#include "module_swerve.h"

#include <stdbool.h>

typedef struct
{
    alg_swerve_t *kinematics;
    module_swerve_t *modules[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];
    module_board_comm_t *board_comm;
} app_chassis_config_t;

typedef struct
{
    app_chassis_config_t config;
    bool initialized;
} app_chassis_t;

bsp_status_t app_chassis_init(app_chassis_t *me, const app_chassis_config_t *config);
void app_chassis_update(app_chassis_t *me, float delta_time_s);

#endif
