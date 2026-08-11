# module_motor — 通用电机基类

该模块统一电机生命周期、反馈、故障状态和注册表。M2006、M3508、GM6020、DM4310 等派生模块通过 `module_motor_t` 与操作表复用公共行为。

## 主要对象

| 对象 | 用途 |
|---|---|
| `module_motor_t` | 电机状态、反馈、超时和派生类操作入口 |
| `module_motor_feedback_t` | 位置、速度、转矩、电流、温度及在线状态 |
| `module_motor_registry_t` | 调用者提供静态数组的电机注册表 |
| `module_motor_pid_t` | 直接封装一个精简后的 `alg_pid_t` |

电机 PID 不再区分位置式/增量式，也不保留控制器联合体。配置类型 `module_motor_pid_config_t` 与 `alg_pid_config_t` 相同；通过 `module_motor_pid_init()`、`module_motor_pid_update()`、`module_motor_pid_reset()` 和 `module_motor_pid_get_terms()` 使用。

## 状态与反馈

```text
DISABLED -> enable -> ENABLED
ENABLED  -> disable -> DISABLED
ENABLED  -> feedback timeout -> FAULT
FAULT    -> clear_fault -> DISABLED
```

周期调用 `module_motor_update()`。收到有效反馈后调用 `module_motor_notify_feedback()` 重置超时计时。业务层通过 `module_motor_get_feedback()` 取得只读反馈，并在使用前检查在线状态及各字段有效标志。

## 分层约束

本模块负责通用电机对象和闭环工具，不决定英雄、步兵或哨兵的控制策略。车型相关的模式切换、目标生成和参数选择应留在项目 App。
