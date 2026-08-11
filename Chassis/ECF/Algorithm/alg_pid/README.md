# alg_pid — 车载 PID 控制

该模块只保留车辆控制中经常使用的三种形态：单环 PID、位置/速度串级 PID、角度串级封装。所有对象由调用者静态分配，周期时间通过 `delta_time_s` 显式传入。

## 单环 PID

`alg_pid_config_t` 提供 `Kp/Ki/Kd`、积分限幅、输出限幅、可选微分低通和“对测量微分”。默认使用条件积分抗饱和：输出已经饱和且积分会继续推向饱和方向时，暂停本次积分。

```c
alg_pid_t speed_pid;
alg_pid_config_t config;
float output;

alg_pid_config_init(&config);
config.proportional_gain = 5.0F;
config.integral_gain = 0.5F;
config.derivative_gain = 0.02F;
config.integral_min = -2.0F;
config.integral_max = 2.0F;
config.output_min = -20.0F;
config.output_max = 20.0F;
config.derivative_on_measurement = true;

alg_pid_init(&speed_pid, &config);
alg_pid_reset(&speed_pid, measured_speed, 0.0F);
alg_pid_update(&speed_pid, target_speed, measured_speed, 0.001F, &output);
```

需要速度、加速度或模型前馈时使用 `alg_pid_update_advanced()`。三个前馈字段均应由上层换算为与控制输出相同的单位，本模块直接求和，不再隐藏额外增益。

## 串级与角度控制

- `alg_pid_cascade_t`：位置外环生成速度目标，速度内环生成执行器输出。
- `position_loop_divider`：降低位置外环运行频率；内环仍每周期执行。
- `alg_pid_angle_t`：为弧度和弧度每秒命名的薄封装，不额外改变控制算法。

## 使用边界

- 不再提供增量式 PID、模糊 PID、在线增益调度和复杂二自由度参数。
- 增益切换、工况判断和轨迹规划属于项目 App，不放进通用 PID。
- 初始化、复位和每次更新都应检查返回状态。

建议验证：正反向阶跃、输出饱和后的积分恢复、零/异常周期、微分噪声、串级外环分频。
