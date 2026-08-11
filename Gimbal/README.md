# SteerWheel Infantry — Gimbal

STM32H723VET6 云台板固件。本目录固定生成云台程序，不需要选择 `GIMBAL/CHASSIS`。

## 本板设备

| 设备 | 总线 | 默认配置 |
|---|---|---|
| Pitch | CAN1 | DM4310，位置/速度控制 |
| Yaw | CAN2 | GM6020，位置外环 + 速度内环 |
| BMI088 | SPI | 姿态与角速度反馈 |
| 左右摩擦轮 | CAN1 | 两个 M3508，速度闭环 |
| 拨弹盘 | CAN1 | 可选 M3508，位置闭环、堵转回退 |
| 板间通信 | CAN2 | 发送 `0x500~0x506`，接收 `0x510~0x516` |
| DR16 | USART | 由安装位置宏决定本地接收或 CAN2 接收 |

## 目录职责

| 路径 | 做什么 |
|---|---|
| `ECF/Algorithm` | PID、LQR、卡尔曼、姿态和运动学；不放车型状态 |
| `ECF/Bsp` | CAN、USART、SPI、USB 等硬件抽象 |
| `ECF/Module` | DM4310、GM6020、M3508、BMI088、DR16、发射机构和板间协议 |
| `ECF/App` | 可复用的云台、IMU、命令、发射和安全控制 |
| `App/config` | 本板引脚映射、电机 ID、方向、零位、PID 初值 |
| `App/robot` | 创建实际设备对象，连接 Module 与 App |
| `App/task` | FreeRTOS 周期调度；只有初始化成功的实例才创建任务 |

## 控制数据流

```text
本地 DR16 或 CAN2 远程 DR16
            ↓
        app_command
            ↓
       app_exchange
       ↙          ↘
 app_gimbal      app_shooter
   ↓                 ↓
Pitch/Yaw       摩擦轮/拨弹盘

BMI088 → app_imu → 云台 IMU 锁定反馈
云台状态 → module_board_comm → CAN2 → 底盘
```

编码器锁定模式直接使用电机位置；IMU 锁定模式使用 `app_imu` 输出。视觉通过 USB 发布目标角度，但裁判系统当前不参与发射许可。

## 配置入口

主要修改 [`App/config/project_config.h`](App/config/project_config.h)：

- `PROJECT_DR16_LOCATION`：DR16 在云台还是底盘；两个工程必须一致。
- `PROJECT_FEEDER_LOCATION`：拨弹盘位置；两个工程必须一致。
- `PROJECT_PITCH_DM4310_*`：Pitch 命令和反馈 ID。
- `PROJECT_YAW_GM6020_ID`、`PROJECT_YAW_GM6020_ENCODER_ZERO`：Yaw ID 与机械零位。
- `PROJECT_GIMBAL_PITCH_MIN_RAD/MAX_RAD`：Pitch 软件限位。
- `PROJECT_YAW_POSITION_*`、`PROJECT_YAW_VELOCITY_*`：Yaw 串级 PID 初值。

## Ozone 调车看什么

| 观察对象 | 重点字段 | 用途 |
|---|---|---|
| `robot_observer` | `init_state`, `failed_step`, `last_status`, `module_error` | 判断初始化卡在哪一步 |
| `robot.devices.pitch_motor` | 电机反馈、目标、在线状态 | Pitch 方向、位置和输出检查 |
| `robot.devices.yaw_motor` | 电机反馈、目标、串级控制器 | Yaw 零位和内外环调试 |
| `robot.devices.bmi088` | 原始加速度、角速度、在线状态 | 检查轴向、符号和传感器通信 |
| `robot.control.imu` | 姿态结果和初始化状态 | 检查 IMU 锁定反馈 |
| `robot.control.gimbal` | 当前命令、反馈模式、锁定状态 | 检查目标值语义和模式切换 |
| `robot.devices.board_comm` | `remote_online`, `chassis_online`, 收发基址 | 检查 CAN2 与远程 DR16 |
| `robot.devices.shooter` | `state`, `friction_ready`, `pending_shots`, `jam_retry_count` | 发射和卡弹回退 |

