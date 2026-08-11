# 项目配置

本目录是 STM32H723 底盘板项目适配层。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

本工程固定为底盘角色，输出 `.build/Debug/steerwheel_chassis.elf`。

## 文件职责

| 文件 | 职责 |
|---|---|
| `project_config.h` | 电机型号、CAN 分配、安装位置、机械参数和调参初值 |
| `board_config.h/.c` | CubeMX HAL 句柄到通用 BSP 对象的绑定 |

## 固定硬件分配

| 板 | CAN1 | CAN2 | CAN3 |
|---|---|---|---|
| 云台板 | Pitch DM4310、摩擦轮及可选拨弹盘 | Yaw GM6020、板间通信 | 预留 |
| 底盘板 | 四个行走 M3508 及可选拨弹盘 | 板间通信 | 四个转向 GM6020 |

`PROJECT_DR16_LOCATION` 和 `PROJECT_FEEDER_LOCATION` 选择 DR16 与拨弹盘安装位置。两块板必须使用一致的安装位置配置。

当前不接入裁判系统。PID 数值和编码器零位是上车前需要通过 Ozone 标定的项目参数。
