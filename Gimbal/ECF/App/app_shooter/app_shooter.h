#ifndef APP_SHOOTER_H
#define APP_SHOOTER_H

#include "bsp_common.h"
#include "module_board_comm.h"
#include "module_shooter.h"

#include <stdbool.h>

typedef struct
{
    module_shooter_t *shooter;
    module_board_comm_t *board_comm;
} app_shooter_config_t;

typedef struct
{
    app_shooter_config_t config;
    bool previous_fire_request;
    bool initialized;
} app_shooter_t;

bsp_status_t app_shooter_init(app_shooter_t *me, const app_shooter_config_t *config);
void app_shooter_update(app_shooter_t *me, float delta_time_s);

#endif
