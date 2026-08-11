# general_framework 架构

当前实例采用双控制板：云台板负责 DM4310 Pitch、GM6020 Yaw、IMU、视觉和摩擦轮；底盘板负责四个 M3508 行走电机与四个 GM6020 转向电机。两板通过 CAN2 共享遥控、云台、底盘和发射状态。ECF 保持共享，板级差异仅位于 `App/config`、`App/robot` 和任务启用策略。

## 总体边界

工程分为“通用 ECF”与“具体机器人项目”两部分：

```text
general_framework/
├── ECF/                         跨项目复用的框架
│   ├── Algorithm/               纯计算、状态估计、控制与运动学
│   ├── Bsp/                     厂商无关的外设对象和公共接口
│   ├── Module/                  可多实例使用的设备和稳定功能模块
│   └── App/                     可跨机器人复用的应用组件
│
├── App/                         当前机器人的项目实例
│   ├── robot/                   机器人组合根、设备与控制装配
│   ├── config/
│   │   ├── project_config.h     角色、参数、CAN ID 与功能选择
│   │   └── board_config.h/.c    H723 HAL 适配、对象存储与回调路由
│   └── task/                    FreeRTOS 周期任务适配器
│
├── Core/、Drivers/              CubeMX 与 STM32 HAL
├── Middlewares/、USB_DEVICE/    FreeRTOS 与 USB 中间件
└── general_framework.ioc        当前板级 CubeMX 工程
```

更换机器人时，项目差异应集中在根目录 `App/` 和 CubeMX 生成的板级文件；
`ECF/` 保持通用。通用 App 与项目 App 的区别不是调用层级，而是复用边界。

## 依赖方向

```text
App/task
   -> App/robot + ECF/App
      -> ECF/Module
      -> ECF/Algorithm
         -> ECF/Bsp
            -> App/config/board_config
               -> STM32 HAL / CubeMX
```

- `ECF/Algorithm` 不依赖 HAL、RTOS、BSP、Module 或 App。
- `ECF/Bsp` 不依赖 Algorithm、Module 或 App，也不直接绑定 CubeMX 全局句柄。
- `ECF/Module` 可以依赖 BSP 和 Algorithm，但不能反向依赖 App。
- `ECF/App` 可以组合 Algorithm、BSP 和 Module，但不能选择具体 HAL 外设实例。
- 根目录 `App` 负责当前机器人的硬件选择、对象存储、参数、角色和初始化顺序。
- `App/task` 只负责周期调度，不承载控制算法和设备协议。
- 下层不能反向调用上层；跨组件数据通过配置、接口和 `app_exchange` 传递。

## 每层放什么

### Algorithm

只接收数值、状态、配置和调用者工作区，不识别 CAN、UART、电机型号或线程。
PID、LQR、滤波、KF/EKF、IMU 姿态、轨迹和底盘运动学都属于这一层。

### BSP

提供厂商无关的外设接口。CAN、USART、SPI 等复杂通信外设使用可多实例对象；
GPIO、EXTI、PWM 使用平台操作表和轻量资源句柄。通用 BSP 不包含 STM32 HAL
头文件，也不直接绑定 `hfdcan1`、`huart5` 等 CubeMX 全局对象。

### Module

表示可注册、可多实例使用的真实设备或稳定功能单元，例如电机、BMI088、
DR16、板间链路和发射机构。Module 保存自身运行状态，通过 BSP 基类访问硬件，
不负责选择具体引脚、总线实例和项目角色。

### 通用 App：`ECF/App`

保存能跨机器人复用的应用组件和控制流程，例如：

- `app_exchange`：组件间强类型数据交换；
- `app_command`：统一命令模型与输入映射流程；
- `app_safety`：通用安全监控和联锁机制；
- `app_chassis`、`app_gimbal`、`app_imu`、`app_shooter`：可配置控制流程；
- `app_vision`：通用视觉通信流程。

通用 App 可以接收 Module/BSP 对象和配置，但不能写死某辆机器人的 CAN ID、
电机数量、任务句柄、HAL 外设或引脚。

### 项目 App：根目录 `App`

保存当前机器人独有的实例与策略：

- `App/config/project_config.h`：机器人角色、PID、CAN ID、机械参数和功能选择；
- `App/config/board_config.h/.c`：H723 外设映射、HAL 操作表、对象存储和回调路由；
- `App/robot/robot.c`：保存唯一的 `robot` 实例，规定初始化和回滚顺序；
- `App/robot/robot_devices.c`：选择并初始化当前板上的 Module/BSP 对象；
- `App/robot/robot_control.c`：把设备注入通用 ECF App，并规定通信、命令映射和视觉更新顺序；
- `App/task/task_*`：FreeRTOS 周期循环，调用对应的通用 App 更新函数。

英雄、步兵、哨兵之间不同的电机数量、遥控映射、模式、安全策略和任务周期，
都应放在项目 App；确认可跨项目复用后，再下沉为 `ECF/App` 的可配置组件。

## 旋转中心约定

底盘坐标统一为 `+x` 向前、`+y` 向左、`+z` 向上，逆时针角速度为正。
旋转中心坐标相对底盘原点给出；指定旋转中心处速度为零时，底盘绕该点纯旋转。

- 舵轮、麦克纳姆轮、全向轮支持任意二维旋转中心。
- 项目当前提供麦轮、全向轮和舵轮解算，不包含差速与 Ackermann 解算。

## 对象与生命周期

- 对象由调用者静态分配，不使用动态内存。
- `init` 初始化全部字段，失败后保持未初始化状态。
- 多态基类首成员为 `super`，基类虚表指针为 `vptr`。
- 派生实现使用静态断言验证 `super` 位于首成员。
- 虚表和驱动操作表为 `static const`。
- 必须操作在构造时校验；可选操作缺失时返回 `UNSUPPORTED`。
- 注册成功后才能使用设备；注销时同时解除总线路由。
- ISR 只完成通知、缓存或置位，解析和控制在任务上下文执行。
- 初始化后的多态对象不能按值复制，只通过指针传递。
- 需要通过 Ozone 观察的机器人对象和诊断量使用稳定的非 `static` 全局符号；控制器状态不放在任务栈上。

## 多态使用边界

只有同一公共接口确实存在两种以上可替换实现时，才使用
`super + vptr + container_of`。典型场景是不同电机型号、不同 BSP 设备实现以及
统一设备生命周期管理。

PID、LQR、滤波、运动学、固定协议工具和单一实现的组合组件继续使用普通结构体
与普通函数，避免为了形式增加虚表和目录层级。

## 项目板级装配

当前边界为：

```text
ECF/Bsp/                       厂商无关外设对象和公共接口
App/config/board_config.h/.c   当前 H723 工程的 HAL 适配与实例装配
Core/、USB_DEVICE/、IOC        CubeMX 生成内容，保持生成位置
```

`board_config.h` 描述资源索引、实例容量和 getter；`board_config.c` 实现 H723 HAL
操作表，持有 BSP 对象存储，将 CubeMX 句柄注入对象，并负责 HAL 回调路由。
更换开发板或 MCU 时替换项目板级适配，Algorithm、Module 和通用 BSP 不重写。

Classic CAN 是 F405 与 H723 可共享的上层边界：F405 bxCAN 端口与 H723 FDCAN
Classic 端口都实现 `bsp_can_driver_ops_t`。只有明确使用 CAN FD 长帧、BRS 或协议
状态时，上层才依赖可选的 `bsp_fdcan_t` 扩展接口。DWT 同样通过
`bsp_dwt_driver_ops_t` 注入寄存器操作。
