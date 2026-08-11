# alg_lqr — 固定增益状态反馈

车载端只执行固定增益控制：

```text
output = feedforward - K × (state - reference)
```

增益矩阵 `K` 应在 MATLAB、Python 或其他离线工具中根据车型和工作点求得，再作为常量放入项目配置。模块不在 MCU 上求解 Riccati 方程，也不包含模型离散化、LQI 增广或角度专用封装。

```c
static const float gain[2] = {8.0F, 1.2F};
static const float output_min[1] = {-10.0F};
static const float output_max[1] = {10.0F};

alg_lqr_t controller;
alg_lqr_config_t config = {
    .state_dimension = 2U,
    .control_dimension = 1U,
    .gain_matrix = gain,
    .control_min = output_min,
    .control_max = output_max,
};

alg_lqr_init(&controller, &config);
alg_lqr_update(&controller, reference, state, feedforward, output);
```

`gain_matrix` 按行主序存放，尺寸为 `control_dimension × state_dimension`。`reference` 和 `feedforward` 可传 `NULL`，分别表示零参考和零前馈。控制输出逐通道限幅。

建议验证：零误差、正负状态误差、多输入矩阵索引、输出限幅、NaN/Inf 拒绝。
