/**
 * @file    task_gimbal.c
 * @brief   云台控制任务 — FreeRTOS 调度适配器
 * @note    云台需要最高的控制频率 (500Hz) 以保证 yaw/pitch 角度跟踪精度。
 *         云台角度目标控制在 ECF/App/app_gimbal/ 中实现。
 */

#include "task_gimbal.h"        // 任务入口声明 (给 freertos.c 用)

#include "app_gimbal.h"         // ECF 框架: 云台角度目标控制更新
#include "robot.h"
#include "project_config.h"     // 项目配置: 任务周期宏
#include "cmsis_os2.h"          // CMSIS-RTOS v2 API

/**
 * @brief  云台任务主循环 (FreeRTOS 线程入口)
 * @param  argument 线程参数 (未使用)
 * @note   周期 APP_GIMBAL_PERIOD_MS (默认 2ms = 500Hz)
 *         云台需要比底盘更高的控制频率以保证角度跟踪精度。
 */
void task_gimbal_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();       // 获取当前系统 tick
    (void)argument;                                     // 参数未使用

    for (;;)
    {
        /* 调用 ECF 框架的云台控制更新: yaw/pitch 双轴角度目标 */
        robot_gimbal_update(robot_get(), (float)APP_GIMBAL_PERIOD_MS * 0.001F);

        wake_tick += APP_GIMBAL_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
