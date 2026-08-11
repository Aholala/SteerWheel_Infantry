# app_shooter — 发射机构控制

双摩擦轮 + 拨弹盘。读取统一发射命令，驱动 `module_shooter` 状态机并发布反馈。

## 用法

```c
app_shooter_config_t cfg = {
    .shooter = &shooter,
    .board_comm = &link,
};
app_shooter_t shooter_app;
app_shooter_init(&shooter_app, &cfg);

// 周期更新
app_shooter_update(&shooter_app, dt);
// 从 app_exchange 读取命令 → module_shooter_update_fire_control → 发布反馈
```

## 火控条件

射击需同时满足：
1. `fire_requested`：操作端请求发射。
2. `friction_enabled`：摩擦轮已被允许启动。
3. `friction_ready`：摩擦轮达到目标速度。
4. `module_shooter` 未处于故障或卡弹锁止状态。
