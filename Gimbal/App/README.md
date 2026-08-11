# 项目 App

本目录保存当前 RoboMaster 机器人实例，移植车型或板卡时主要修改这里。

| 目录 | 职责 |
|---|---|
| `config` | 项目参数、板级资源映射和 HAL 适配 |
| `robot` | 当前机器人对象、设备装配、控制装配和初始化回滚 |
| `task` | FreeRTOS 周期调度适配器 |

依赖方向固定为：

```text
task -> robot -> ECF/App -> ECF/Module -> ECF/Bsp + ECF/Algorithm
```

命名约定：

- 项目组合根使用 `robot_*`，不使用容易与 `ECF/App/app_*` 混淆的 `app_robot`。
- FreeRTOS 入口使用 `task_<name>.c/.h`，直接放在 `task/`，不为单个源文件建立子目录。
- 配置入口统一为 `config/README.md`。
- 目录说明统一命名为 `README.md`；短小重复说明合并到父目录，不为每个文件建立文档。
