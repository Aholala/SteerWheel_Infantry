# ECF

ECF 是面向 RoboMaster 的可移植框架部分，不保存某一台机器人的 CAN ID、板级句柄或任务实例。

| 目录 | 职责 | 命名规则 |
|---|---|---|
| `Algorithm` | 纯数值、滤波、估计、控制和运动学 | `alg_<name>` |
| `Bsp` | 厂商无关的外设接口 | `bsp_<name>` |
| `Module` | 电机、传感器和通信设备对象 | `module_<name>` |
| `App` | RoboMaster 通用功能流程 | `app_<name>` |
| `Docs` | 原厂硬件资料，不参与编译 | 保留原厂文件名 |

每个公开模块目录使用一个 `README.md` 作为入口。型号或子功能补充文档使用小写名称，例如 `m3508.md`、`dm4310.md`、`health.md`，不再使用 `README_xxx.md`。

ECF 代码使用 C11、静态内存和显式依赖。实例选择、参数和 FreeRTOS 调度位于根目录 `App`。
