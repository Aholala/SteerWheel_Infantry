# SteerWheel Infantry

STM32H723VET6 双板舵轮步兵工程。云台和底盘分别构建、分别烧录，不再通过构建参数切换板型。

完整的云台、底盘、遥控映射、CAN2、构建和调车说明见 [`PROJECT_GUIDE.md`](PROJECT_GUIDE.md)。

## 两个固件

| 目录 | 输出 | 本板职责 |
|---|---|---|
| [`Gimbal`](Gimbal/) | `steerwheel_gimbal.elf` | Pitch DM4310、Yaw GM6020、BMI088、双摩擦轮、可选拨弹盘、视觉 |
| [`Chassis`](Chassis/) | `steerwheel_chassis.elf` | 四个 M3508 驱动轮、四个 GM6020 转向电机、可选拨弹盘 |

两个工程都包含一份 ECF，以便独立拉取、构建和移植。通用算法、BSP、设备驱动放在 `ECF`；本车的引脚、电机 ID、控制参数、对象装配和 Task 放在 `App`。

## 板间 CAN2

板间协议采用独立收发 ID 段：

| 方向 | CAN ID |
|---|---|
| 云台 → 底盘 | `0x500~0x506` |
| 底盘 → 云台 | `0x510~0x516` |

消息包含 DR16、云台姿态、底盘状态和发射状态。多帧数据使用序列号完整组装，超过配置时间未更新则判定离线。两边的 `PROJECT_DR16_LOCATION`、`PROJECT_FEEDER_LOCATION` 和通信 ID 必须保持一致。
