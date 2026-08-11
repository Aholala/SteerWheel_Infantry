# App — 业务应用层

提供 RoboMaster 各子系统可复用的控制流程。具体设备选择和整机装配位于根目录 `App/robot`。

## 模块列表

| 模块 | 职责 |
|------|------|
| `app_command` | 遥控器→云台/底盘/发射机构命令映射 |
| `app_gimbal` | 云台角度目标控制（IMU/编码器反馈） |
| `app_chassis` | 底盘运动（跟随/旋转/无力模式） |
| `app_shooter` | 发射机构状态机 + 火控 |
| `app_imu` | IMU 读数 + 姿态估计 + 坐标系变换 |
| `app_vision` | USB 视觉通信：mode/ID 协议 |
| `app_safety` | 安全监控（看门狗、遥控失联、电机健康） |
| `app_exchange` | 模块间数据交换（共享内存，零拷贝） |

## 初始化

所有 `app_*_init()` 返回 `bsp_status_t`（不再返回 `bool`）：

这些组件由 `App/robot/robot_control.c` 根据当前机器人配置完成初始化。

错误信息通过全局寄存器 `bsp_error_read()` 获取。

## 依赖方向

```
Task（FreeRTOS 入口）→ App/robot（实例装配）→ ECF/App（本层）→ Module + Algorithm + BSP
```

App 不依赖 Task，Task 只转发到 `app_*_update()`。
