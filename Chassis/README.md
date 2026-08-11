# SteerWheel Infantry — Chassis

STM32H723VET6 舵轮底盘板固件。本目录固定生成底盘程序，不需要选择 `GIMBAL/CHASSIS`。

## 本板设备

| 设备 | 总线 | 默认配置 |
|---|---|---|
| 四个驱动轮 | CAN1 | M3508，速度闭环 |
| 四个转向电机 | CAN3 | GM6020，位置外环 + 速度内环 |
| 可选拨弹盘 | CAN1 | M3508，位置闭环、堵转回退 |
| 板间通信 | CAN2 | 发送 `0x510~0x516`，接收 `0x500~0x506` |
| DR16 | USART | 由安装位置宏决定本地接收或 CAN2 接收 |

## 目录职责

| 路径 | 做什么 |
|---|---|
| `ECF/Algorithm` | PID、舵轮运动学和通用数学算法 |
| `ECF/Bsp` | CAN、USART 等硬件抽象 |
| `ECF/Module` | M3508、GM6020、单舵轮、DR16、发射机构和板间协议 |
| `ECF/App` | 可复用的底盘、命令和安全控制 |
| `App/config` | 本板 CAN、电机 ID、方向、零位、底盘尺寸和 PID 初值 |
| `App/robot` | 创建八个电机、四个舵轮模块和底盘 App 实例 |
| `App/task` | FreeRTOS 周期调度；初始化成功后才启用底盘任务 |

## 控制数据流

```text
本地 DR16 或 CAN2 远程 DR16
            ↓
        app_command
            ↓
       app_exchange
            ↓
        app_chassis
            ↓
        alg_swerve
            ↓
  四个 module_swerve
      ↙             ↘
4×M3508           4×GM6020

云台姿态 ← CAN2，用于普通、小陀螺和跟随云台坐标变换
```

底盘模式包括无力、普通、小陀螺和跟随云台；静止自锁由 `app_chassis` 下发到四个舵轮模块。

## 配置入口

主要修改 [`App/config/project_config.h`](App/config/project_config.h)：

- `PROJECT_DR16_LOCATION`、`PROJECT_FEEDER_LOCATION`：必须与云台工程一致。
- `PROJECT_DRIVE_*_ID`：四个 M3508 的 CAN ID。
- `PROJECT_STEERING_*_ID`：四个 GM6020 的 CAN ID。
- `PROJECT_CHASSIS_HALF_WHEELBASE_M`、`PROJECT_CHASSIS_HALF_TRACK_M`：轮组中心坐标。
- `PROJECT_CHASSIS_WHEEL_RADIUS_M`、`PROJECT_CHASSIS_DRIVE_REDUCTION_RATIO`：速度换算。
- `PROJECT_DRIVE_VELOCITY_*`：驱动轮速度环。
- `PROJECT_STEERING_POSITION_*`、`PROJECT_STEERING_VELOCITY_*`：转向串级环。

四个转向电机的机械零位必须逐个标定。当前配置中的 PID 和几何值只是安全初值。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

输出：`.build/Debug/steerwheel_chassis.elf`。

## Ozone 调车看什么

| 观察对象 | 重点字段 | 用途 |
|---|---|---|
| `robot_observer` | `init_state`, `failed_step`, `last_status`, `module_error` | 定位初始化失败 |
| `robot.devices.drive_motors[0..3]` | 速度反馈、目标、电流输出、在线状态 | 四个驱动轮逐一核对 |
| `robot.devices.steering_motors[0..3]` | 位置、速度、目标和控制输出 | 标定转向零位和串级 PID |
| `robot.devices.swerve_modules[0..3]` | 期望轮速、期望转角和反向优化状态 | 检查单舵轮组合逻辑 |
| `robot.devices.chassis_kinematics` | 几何和运动学结果 | 检查四轮目标分配 |
| `robot.control.chassis` | 当前模式、命令和初始化状态 | 检查普通/小陀螺/跟随/自锁 |
| `robot.devices.board_comm` | `remote_online`, `gimbal_online`, 收发基址 | 检查 CAN2、云台角和远程 DR16 |
| `robot.devices.shooter` | 拨弹状态和卡弹计数 | 仅拨弹盘安装在底盘时观察 |

调参顺序：单个 GM6020 速度环 → 位置环 → 四个转向零位 → 单个 M3508 速度环 → 四轮同向测试 → 平移 → 旋转 → 小陀螺与跟随。第一次闭环必须架空轮组并限制输出。

## 上电检查

1. 架空底盘，遥控保持无力。
2. 确认八个电机全部在线且 ID 没有重复。
3. 手动转动每个转向电机，标记数组下标和实车轮位。
4. 分别给四个舵轮很小的目标，确认转向和驱动方向。
5. 检查 CAN2 的 `gimbal_online` 和云台 Yaw 数据，再测试跟随模式。
6. 最后落地低速测试自锁、普通、跟随和小陀螺。

