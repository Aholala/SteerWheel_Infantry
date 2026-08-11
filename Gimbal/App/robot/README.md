# Robot 实例层

本目录组合一台双板 RoboMaster 舵轮车。ECF 组件不依赖本目录，FreeRTOS Task 只调度这里已经初始化的实例。

## 双板职责

| 固件目标 | 本地设备 | 本地控制 |
|---|---|---|
| `GIMBAL` | Pitch DM4310（CAN1）、Yaw GM6020（CAN2）、BMI088、两路摩擦轮、可选 DR16/拨弹盘 | 云台、IMU、视觉、发射 |
| `CHASSIS` | 四个 M3508（CAN1）、四个转向 GM6020（CAN3）、可选 DR16/拨弹盘 | 舵轮底盘、可选拨弹控制 |

CAN2 同时承担板间通信。云台发送段为 `0x500~0x506`，底盘发送段为 `0x510~0x516`；两板只接收对端 ID 段，与 DJI 电机帧分开。

摩擦轮始终安装在云台板；`PROJECT_FEEDER_LOCATION` 只切换拨弹盘位置。拨弹盘位于底盘时，底盘通过 CAN2 获取云台摩擦轮到速状态后才允许送弹，卡弹检测、回退和有限次数重试仍由同一个 `module_shooter` 状态机处理。

## 文件

| 文件 | 职责 |
|---|---|
| `robot.c` | 全局 `robot`、初始化状态和失败记录 |
| `robot_devices.c` | 当前板设备与 CAN/DR16/板间通信装配 |
| `robot_control.c` | 当前板 App 实例初始化与命令更新 |
| `robot.h` | `robot_t`、`robot_control_t` 和 Ozone 可见符号 |

## 实例规则

`app_chassis_t`、`app_gimbal_t`、`app_imu_t`、`app_shooter_t` 都存放在 `robot.control`，不再使用隐藏的文件级单例。Task 只有在对应实例初始化成功后才创建，避免周期性空跑。

## Ozone

Debug 构建使用 `-O0 -g3`。推荐观察：

- `robot.devices`：本板设备和通信状态；
- `robot.control`：底盘、云台、IMU、发射实例及配置；
- `robot_observer`：初始化阶段、失败步骤和更新计数。
