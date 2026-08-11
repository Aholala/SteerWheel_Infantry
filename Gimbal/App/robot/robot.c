/**
 * @file robot.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 机器人主模块初始化与全局实例管理
 * @version 1.0
 * @date 2026-08-11
 * @copyright Copyright (c) 2026
 *
 * @details 本文件提供机器人实例的全局变量、错误记录函数以及
 *          顶层初始化入口。初始化分两步：设备层(robot_devices_init)
 *          和控制层(robot_control_init)，并记录状态到观察者结构体。
 */

#include "robot.h"
#include "robot_internal.h"

/** 全局机器人实例 */
robot_t robot;
/** 全局机器人状态观察者（用于诊断） */
robot_observer_t robot_observer;

/**
 * @brief 记录机器人初始化失败信息
 *
 * @param status        BSP 状态码
 * @param step          失败步骤描述
 * @param module_error  模块错误码（扩展信息）
 *
 * @note 该函数同时调用 bsp_error_record 记录到底层错误日志。
 */
void robot_record_failure(bsp_status_t status, const char *step, int32_t module_error)
{
    robot_observer.init_state = ROBOT_INIT_STATE_FAILED;
    robot_observer.last_status = status;
    robot_observer.module_error = module_error;
    robot_observer.failed_step = step;
    robot_observer.initialized = false;
    bsp_error_record(status, step, module_error);
}

/**
 * @brief 机器人顶层初始化
 *
 * @param me  机器人实例指针
 * @return bsp_status_t  BSP_STATUS_OK 成功，其他为失败
 *
 * @note 执行顺序：
 *       1. 参数校验
 *       2. 清零实例与观察者
 *       3. 调用 robot_devices_init 初始化设备层
 *       4. 调用 robot_control_init 初始化控制层
 *       5. 若任何步骤失败，则调用 robot_devices_deinit 回滚
 *       6. 最终标记 initialized = true 并更新观察者状态
 */
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

/**
 * @brief 获取全局机器人实例指针
 *
 * @return robot_t*  返回 &robot
 */
robot_t *robot_get(void)
{
    return &robot;
}