#ifndef ROBOT_INTERNAL_H
#define ROBOT_INTERNAL_H

#include "robot.h"

bsp_status_t robot_devices_init(robot_t *me);
void robot_devices_deinit(robot_t *me);
bsp_status_t robot_control_init(robot_t *me);

void robot_record_failure(bsp_status_t status, const char *step, int32_t module_error);

#endif
