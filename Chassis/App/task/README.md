# FreeRTOS Task 适配层

Task 文件保持扁平，因为每个任务只有一个入口 `.c` 和一个声明 `.h`。它们只负责固定周期调度，不保存设备或控制状态。

| Task | 云台板 | 底盘板 | 周期调用 |
|---|---:|---:|---|
| `task_command` | 是 | 是 | 本地 DR16 或板间遥控命令更新 |
| `task_safety` | 是 | 是 | 失联与安全监控 |
| `task_gimbal` | 是 | 否 | `robot.control.gimbal` |
| `task_imu` | 是 | 否 | `robot.control.imu` |
| `task_chassis` | 否 | 是 | `robot.control.chassis` |
| `task_shooter` | 按本地发射实例 | 按本地发射实例 | `robot.control.shooter` |

`Core/Src/freertos.c` 根据板角色和实例的 `initialized` 状态创建任务。未初始化的控制对象不会生成空跑线程。

