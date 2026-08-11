/**
 * @file    task_command.c
 * @brief   命令与通信任务 — FreeRTOS 调度适配器
 * @note    每个周期内按顺序调用三个更新:
 *          robot_command_update() 统一执行通信、命令映射和视觉更新。
 */

#include "task_command.h"       // 任务入口声明 (给 freertos.c 用)

#include "project_config.h"     // 项目配置: 任务周期宏
#include "robot.h"              // 项目实例: 机器人组合根
#include "cmsis_os2.h"          // CMSIS-RTOS v2 API

/**
 * @brief  命令任务主循环 (FreeRTOS 线程入口)
 * @param  argument 线程参数 (未使用)
 * @note   周期 APP_COMMAND_PERIOD_MS (默认 5ms = 200Hz)
 */
void task_command_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();       // 获取当前系统 tick
    (void)argument;                                     // 参数未使用

    for (;;)
    {
        robot_command_update(robot_get(), APP_COMMAND_PERIOD_MS);

        wake_tick += APP_COMMAND_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
