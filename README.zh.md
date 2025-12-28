# Atom Deploy 中文手册

[![GitHub stars](https://img.shields.io/github/stars/Dobot-Arm/atom-deploy?style=social)](https://github.com/Dobot-Arm/atom-deploy)
[![OS Ubuntu](https://img.shields.io/badge/OS-Ubuntu%2020.04%2F22.04-orange)](#)
[![Compiler](https://img.shields.io/badge/Compiler-gcc%209%2B%20%7C%20clang%2012%2B-blue)](#)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-green)](#)

> English version is available in [README.md](README.md).

Atom Deploy 是DOBOT Atom 足式机器人在真实机器人上运行强化学习策略的部署程序。它负责：

- 管理底层桥接（`Atom::Bridge`）与机器人本体/手柄的数据同步；
- 以 `LoopFunc` 实现 500 Hz 控制循环与 100 Hz 推理循环；
- 通过有限状态机（`robot_fsm`）切换被动、默认、初始化和模型运行等状态；
- 调用 ONNX Runtime 加载 `policy/model/atom.onnx`，执行策略，输出腿/臂关节命令。

---

## 目录

1. [项目结构](#项目结构)
2. [环境与依赖](#环境与依赖)
3. [安装流程](#安装流程)
4. [配置说明](#配置说明)
5. [构建与运行](#构建与运行)
6. [控制映射](#控制映射)
7. [调试与故障排查](#调试与故障排查)
8. [二次开发提示](#二次开发提示)

---

## 项目结构

```
deploy/
├── CMakeLists.txt           # 构建脚本，定义依赖与目标
├── README.md                # 英文使用指南
├── README.zh.md             # 中文使用指南（本文件）
├── include/
│   ├── atom_deploy.hpp      # AtomDeploy 类声明、FSM/Loop 定义
│   ├── loop/                # LoopFunc 实现
│   ├── robot_fsm/           # 状态机定义
│   └── robot_model/         # RL 观测与模型封装
├── policy/
│   ├── config.yaml          # 控制 & 策略参数
│   └── model/atom.onnx      # RL 策略文件（需自行放置）
└── src/atom_deploy.cpp      # 主程序：状态读取、FSM 调度、模型推理
```

主要逻辑集中在 `AtomDeploy`，负责桥接状态、执行 FSM、写回命令及维护 RL 输出队列。`CMakeLists.txt:1` 指定 C++17 标准，链接 ONNX Runtime、TBB、Boost::system 以及 `dobot_atom_sdk` 提供的静态库。

---

## 环境与依赖

- 操作系统：Ubuntu 20.04/22.04（需支持 `gcc >= 9` 或 `clang >= 12`）
- CMake ≥ 3.16
- 必备库
  - ONNX Runtime（建议 v1.19.0，源码构建以获得 GPU/自定义算子支持）
  - DOBOT Atom SDK（包含桥接、IDL、DDS 依赖）
  - Boost（最少 `system` 组件）
  - Intel TBB、pthread（随系统提供）
- 设备：需连接真实 Atom 机器人以及官方手柄，或在 SDK 提供的仿真环境中测试。

> **注意**：仓库内未包含 `dobot_atom_sdk`，需根据 README 指引额外获取。

---

## 安装流程

1. **克隆本仓库**
   ```bash
   git clone http://gitlab.dobot.com/atom-locomotion/deploy.git
   cd deploy
   ```

2. **安装 ONNX Runtime（源码方式）**
   ```bash
   git clone https://github.com/microsoft/onnxruntime
   cd onnxruntime
   git checkout v1.19.0
   ./build.sh --config Release --build_shared_lib --parallel --allow_running_as_root
   cd onnxruntime/build/Linux/Release/
   sudo make install
   ```

3. **安装 DOBOT ATOM SDK**
   ```bash
   git clone https://github.com/Dobot-Arm/dobot_atom_sdk.git
   cd dobot_atom_sdk
   rm -rf build && mkdir build
   cd build
   cmake ..
   make
   ```
   将 SDK 根目录路径记录在 `SDK_DIR`（`CMakeLists.txt` 默认指向仓库内 `dobot_atom_sdk` 子目录，如需自定义请修改）。

4. **安装 Boost**
   ```bash
   sudo apt-get update
   sudo apt-get install libboost-all-dev
   ```

5. **其他系统依赖**
   - `sudo apt-get install libtbb-dev`
   - 若使用 GPU，可按需安装 CUDA/cuDNN，并在编译 ONNX Runtime 时启用相关选项。

---

## 配置说明

所有策略与控制参数位于 `policy/config.yaml`，默认读取 `atom` 节点。常用字段示例如下：

| 字段 | 含义 |
| ---- | ---- |
| `dt` / `inference_dt` | 控制循环（500 Hz）与推理循环（100 Hz）周期 |
| `fixed_kp`, `fixed_kd` | 腿/臂每个关节的位置/速度增益 |
| `torque_limits` | 安全扭矩上限 |
| `default_dof_pos` | 复位/起始姿态 |
| `model_name` | ONNX 模型文件名（默认 `atom.onnx`）|
| `num_one_step_observations`, `num_history` | 策略观测维度与历史长度 |
| `clip_obs`, `clip_actions`, `action_scale` | 观测裁剪、动作裁剪及缩放 |
| `max_speed_x/y`, `max_yaw_rate`, `deadzone` | 手柄输入缩放与死区，用于 `GetState()` 里的滤波逻辑 |

如需切换策略，只需将新的 `*.onnx` 文件放入 `policy/model/` 并设置 `model_name`。若添加新的 YAML 字段，可在 `AtomDeploy::ReadYaml` 后通过 `params.Get<T>()` 访问。

---

## 构建与运行

1. **配置构建目录**
   ```bash
   mkdir -p build && cd build
   cmake .. \
     -DCMAKE_BUILD_TYPE=Release \
     -DDOBOT_SDK_DIR=/path/to/dobot_atom_sdk   # 可选：覆盖默认 SDK 路径
   ```

2. **编译**
   ```bash
   make -j$(nproc)
   ```

3. **运行**
   ```bash
   ./atom_deploy
   ```
   进程会注册 `SIGINT`（Ctrl+C）处理器，循环执行 `AtomDeploy::RobotControl()` 与 `AtomDeploy::RunModel()`。请确保手柄与机器人都已上电且桥接正常。

---

## 控制映射

默认手柄映射（示例）：

| 操作                      | 按键      | 说明                                             |
| ------------------------- | --------- | ------------------------------------------------ |
| 被动模式                  | `SELECT`  | 切换到 `StatePassive`，电机进入被动状态          |
| 回默认关节位姿            | `A`       | `StateDefaultPos`，机器人恢复 `default_dof_pos` |
| 初始化姿态                | `B`       | `StateInitPose`，完成站起/初始化动作             |
| 运行策略（模型）          | `RB + ↑`  | `StateModel`，进入 RL 控制                       |
| 自由行走输入              | 左摇杆    | `lin_vel` 映射，内部使用低通与死区过滤           |
| 自由转向输入              | 右摇杆 X  | `yaw_vel` 映射                                   |

> 真实按键名称请以手柄固件映射为准，`AtomDeploy::GetState()` 中可看到对应布尔变量（`button_A_`, `button_B_`, `button_R1U_`, `button_SELECT_` 等）。

---

## 调试与故障排查

1. **ONNX Runtime 找不到**
   - 确认 `libonnxruntime.so` 已安装到系统路径或设置 `LD_LIBRARY_PATH`。
   - 使用 `ldd ./atom_deploy` 检查动态库依赖。

2. **SDK 头文件/库缺失**
   - 检查 `CMakeLists.txt` 中 `SDK_DIR` 是否指向正确目录。
   - 运行 `cmake .. -DSDK_DIR=/absolute/path` 明确指定。

3. **FSM 无响应**
   - 确认手柄输入是否被死区滤除，可在 `AtomDeploy::GetState()` 添加日志。
   - 查看 `robot_fsm` 状态转换是否覆盖新事件。

4. **推理无输出**
   - `rl_init_done` 需在 FSM 中设置为 `true`，否则 `RunModel()` 不会调用 `Forward()`。
   - 检查 `policy/model/atom.onnx` 是否存在且版本匹配。

---

## 二次开发提示

- 扩展动作空间：修改 `policy/config.yaml` 中的 `num_of_policy_dofs`/`action_dim`，同步调整 `RobotModel` 的观测/动作处理。
- 新增控制状态：在 `robot_fsm` 中添加状态/事件，并在 `AtomDeploy::RobotControl()` 中处理新的 `FsmChange`。
- 记录数据：利用 `output_dof_pos_queue` / `output_dof_vel_queue` 可快速接入日志线程。
- 多策略切换：可以在 `policy/model/` 放置多份 ONNX，通过 RPC 或按键更换 `model_name` 后重新加载。

如需进一步定制，请结合 `include/atom_deploy.hpp` 与 `src/atom_deploy.cpp` 查看实现细节。欢迎根据自身项目需要补充更多自动化脚本/监控工具。
