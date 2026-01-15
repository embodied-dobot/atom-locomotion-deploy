# Atom Deploy
[English](README.md) | [中文](README.zh.md)
[![GitHub stars](https://img.shields.io/github/stars/Dobot-Arm/atom-deploy?style=social)](https://github.com/Dobot-Arm/atom-deploy)
[![OS Ubuntu](https://img.shields.io/badge/OS-Ubuntu%2020.04%2F22.04-orange)](#)
[![Compiler](https://img.shields.io/badge/Compiler-gcc%209%2B%20%7C%20clang%2012%2B-blue)](#)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-green)](#)

> For Chinese instructions, please read [README.zh.md](README.zh.md).

Atom Deploy is the runtime that executes reinforcement-learning policies on the DOBOT Atom legged robot. It:

- synchronizes robot/base/joystick state through `Atom::Bridge`;
- runs a 500 Hz control loop and a 100 Hz inference loop via `LoopFunc`;
- drives the FSM (`robot_fsm`) to switch passive, default, init and model-running modes;
- loads `policy/model/atom.onnx` with ONNX Runtime, producing leg/arm commands.

---

## Table of Contents

1. [Project Layout](#project-layout)
2. [Environment & Dependencies](#environment--dependencies)
3. [Installation Steps](#installation-steps)
4. [Configuration Guide](#configuration-guide)
5. [Build & Run](#build--run)
6. [Control Mapping](#control-mapping)
7. [Debugging & Troubleshooting](#debugging--troubleshooting)
8. [Development Tips](#development-tips)

---

## Project Layout

```
deploy/
├── CMakeLists.txt           # Build script and dependency wiring
├── README.md                # English manual (this file)
├── README.zh.md             # Chinese manual
├── include/
│   ├── atom_deploy.hpp      # AtomDeploy declaration, FSM/loop interfaces
│   ├── loop/                # LoopFunc implementation
│   ├── robot_fsm/           # FSM definition
│   └── robot_model/         # RL observation/model wrapper
├── policy/
│   ├── config.yaml          # Control & policy parameters
│   └── model/atom.onnx      # RL policy file (supply your own)
└── src/atom_deploy.cpp      # Main logic: state read, FSM dispatch, inference
```

Most orchestration lives inside `AtomDeploy`: it bridges states, runs the FSM, publishes commands, and queues RL outputs. `CMakeLists.txt:1` enforces C++17 and links ONNX Runtime, TBB, Boost::system, and the static libs inside `dobot_atom_sdk`.

---

## Environment & Dependencies

- OS: Ubuntu 20.04/22.04 with `gcc >= 9` or `clang >= 12`
- CMake ≥ 3.16
- Required libraries
  - ONNX Runtime (recommend v1.19.0 built from source for GPU/custom ops)
  - DOBOT Atom SDK (bridge, IDL, DDS stack)
  - Boost (at least `system`)
  - Intel TBB, pthread (system packages)
- Hardware: a real Atom robot plus the official gamepad, or the SDK’s simulator.

> **Note**: `dobot_atom_sdk` is not vendored; follow the instructions below to obtain it.

---

## Installation Steps

1. **Clone this repo**
   ```bash
   git clone http://gitlab.dobot.com/atom-locomotion/deploy.git
   cd deploy
   ```

2. **Build ONNX Runtime from source**
   ```bash
   git clone https://github.com/microsoft/onnxruntime
   cd onnxruntime
   git checkout v1.19.0
   ./build.sh --config Release --build_shared_lib --parallel --allow_running_as_root
   cd build/Linux/Release/
   sudo make install
   ```

3. **Build the DOBOT ATOM SDK**
   ```bash
   git clone https://github.com/Dobot-Arm/dobot_atom_sdk.git
   cd dobot_atom_sdk
   rm -rf build && mkdir build
   cd build
   cmake ..
   make
   ```
   Record the SDK root path in `SDK_DIR`. `CMakeLists.txt` expects it under `dobot_atom_sdk` inside this repo; change it if you keep the SDK elsewhere.

4. **Install Boost**
   ```bash
   sudo apt-get update
   sudo apt-get install libboost-all-dev
   ```

5. **Other system packages**
   - `sudo apt-get install libtbb-dev`
   - Install CUDA/cuDNN if you plan to enable GPU kernels when building ONNX Runtime.

---

## Configuration Guide

All policy/control knobs live in `policy/config.yaml` under the `atom` node. Common keys:

| Key | Description |
| --- | ----------- |
| `dt` / `inference_dt` | Control loop (500 Hz) and inference loop (100 Hz) periods |
| `fixed_kp`, `fixed_kd` | Per-joint leg/arm gains |
| `torque_limits` | Command torque safety limits |
| `default_dof_pos` | Reset/start posture |
| `model_name` | ONNX filename (default `atom.onnx`) |
| `num_one_step_observations`, `num_history` | Policy observation dimension and history length |
| `clip_obs`, `clip_actions`, `action_scale` | Observation clipping, action clipping & scaling |
| `max_speed_x/y`, `max_yaw_rate`, `deadzone` | Joystick scaling and dead zone used in `GetState()` |

To swap policies, drop the new `*.onnx` under `policy/model/` and update `model_name`. Any new YAML fields become available through `params.Get<T>()` after `AtomDeploy::ReadYaml`.

---

## Build & Run

1. **Configure**
   ```bash
   mkdir -p build && cd build
   cmake .. \
     -DCMAKE_BUILD_TYPE=Release \
     -DDOBOT_SDK_DIR=/path/to/dobot_atom_sdk   # optional override
   ```

2. **Compile**
   ```bash
   make -j$(nproc)
   ```

3. **Execute**
   ```bash
   ./atom_deploy
   ```
   The binary installs a `SIGINT` (Ctrl+C) handler and continuously runs `AtomDeploy::RobotControl()` plus `AtomDeploy::RunModel()`. Ensure the robot and controller are powered and the bridge is online.

---

## Control Mapping

Default gamepad layout (example):

| Action                    | Button    | Description |
| ------------------------- | --------- | ----------- |
| Passive mode              | `SELECT`  | Switch to `StatePassive`, motors relax |
| Default joint posture     | `A`       | `StateDefaultPos`, return to `default_dof_pos` |
| Init pose                 | `B`       | `StateInitPose`, perform stand-up/init motion |
| Run policy                | `RB + ↑`  | `StateModel`, start RL control |
| Linear velocity commands  | Left stick | Maps to `lin_vel`, filtered by LP + dead zone |
| Yaw commands              | Right stick X | Maps to `yaw_vel` |

> Exact button names depend on the firmware mapping. Inspect `AtomDeploy::GetState()` for the boolean flags (`button_A_`, `button_B_`, `button_R1U_`, `button_SELECT_`, …).

---

## Debugging & Troubleshooting

1. **ONNX Runtime not found**
   - Ensure `libonnxruntime.so` is installed in a library path or export `LD_LIBRARY_PATH`.
   - Run `ldd ./atom_deploy` to check dynamic dependencies.

2. **SDK headers/libs missing**
   - Verify `SDK_DIR` in `CMakeLists.txt` or pass `-DDOBOT_SDK_DIR=/absolute/path`.
   - Re-run CMake after adjusting the path.

3. **FSM unresponsive**
   - Confirm joystick inputs are not wiped by the dead zone; add logs in `GetState()`.
   - Double-check `robot_fsm` transitions cover the new events you expect.

4. **No policy output**
   - `rl_init_done` must be flipped to `true` inside the FSM; otherwise `RunModel()` never calls `Forward()`.
   - Confirm `policy/model/atom.onnx` exists and matches the trained policy version.

---

## Development Tips

- **Expand action space**: adjust `num_of_policy_dofs` / `action_dim` in `policy/config.yaml`, then update `RobotModel` observation/action handling.
- **Add FSM states**: extend `robot_fsm`, introduce new events, and map them in `AtomDeploy::RobotControl()`.
- **Log data**: consume `output_dof_pos_queue` / `output_dof_vel_queue` from another thread to persist trajectories.
- **Policy hot-swap**: store multiple ONNX files under `policy/model/` and reload by updating `model_name` via RPC or input commands.

For deeper customization, read through `include/atom_deploy.hpp` and `src/atom_deploy.cpp`. Feel free to layer additional tooling or automation based on your deployment workflow.
