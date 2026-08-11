# app_gimbal — 云台控制

偏航（Yaw）和俯仰（Pitch）双轴云台。应用层统一提交角度目标，电机模块完成角度、速度和电流串级闭环；反馈可选择 IMU 或编码器。

## 用法

```c
app_gimbal_config_t cfg = {
    .yaw_motor   = &yaw_gm6020,
    .pitch_motor = &pitch_gm6020,
    .target_tolerance_rad = 0.01f,
};
app_gimbal_t gimbal;
app_gimbal_init(&gimbal, &cfg);

// 周期更新
app_gimbal_update(&gimbal, dt);
// 从 app_exchange 读取命令 → 控制电机 → 发布反馈
```

## 控制约定

云台电机应在实例配置中选择角度模式，`module_motor_set_target()` 的输入单位固定为 rad。应用层不再运行时切换 PID/LQR，避免把 LQR 的电流或力矩控制量误当成角度目标。

通用 `alg_lqr` 仍保留在算法层；项目若确实使用 LQR，应为其提供单位明确的电流或力矩执行接口，并在具体机器人 App 中组合。

## 反馈模式

| 模式 | 说明 |
|------|------|
| `APP_GIMBAL_FEEDBACK_IMU` | IMU 姿态作反馈（绝对角度） |
| `APP_GIMBAL_FEEDBACK_ENCODER` | 电机编码器作反馈（相对角度） |
