#include "robot.h"

#include "robot_internal.h"

robot_t robot;
robot_observer_t robot_observer;

void robot_record_failure(bsp_status_t status, const char *step, int32_t module_error)
{
    robot_observer.init_state = ROBOT_INIT_STATE_FAILED;
    robot_observer.last_status = status;
    robot_observer.module_error = module_error;
    robot_observer.failed_step = step;
    robot_observer.initialized = false;
    bsp_error_record(status, step, module_error);
}

bsp_status_t robot_init(robot_t *me)
{
    bsp_status_t status;

    if (me == NULL)
    {
        robot_record_failure(BSP_STATUS_INVALID_ARGUMENT, "robot", 0);
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (me->initialized)
    {
        return BSP_STATUS_BUSY;
    }

    *me = (robot_t){0};
    robot_observer = (robot_observer_t){0};
    robot_observer.init_state = ROBOT_INIT_STATE_DEVICES;

    status = robot_devices_init(me);
    if (status != BSP_STATUS_OK)
    {
        robot_devices_deinit(me);
        return status;
    }

    robot_observer.init_state = ROBOT_INIT_STATE_CONTROL;
    status = robot_control_init(me);
    if (status != BSP_STATUS_OK)
    {
        robot_devices_deinit(me);
        return status;
    }

    me->initialized = true;
    robot_observer.init_state = ROBOT_INIT_STATE_READY;
    robot_observer.last_status = BSP_STATUS_OK;
    robot_observer.dr16_is_local = me->devices.dr16_is_local;
    robot_observer.initialized = true;
    return BSP_STATUS_OK;
}

robot_t *robot_get(void)
{
    return &robot;
}
