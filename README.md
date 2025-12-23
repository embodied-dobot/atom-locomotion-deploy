# Atom Deploy

A complete guide covering environment setup, dependency installation, building, running, and controls.

---

## Table of Contents

* [🧪 Source Code](#-source-code)
* [📦 Dependencies & Environment](#-dependencies--environment)
  * [ONNX Runtime](#onnx-runtime-from-source)
  * [DOBOT ATOM SDK](#dobot-atom-sdk)
  * [Boost](#boost)
* [🔧 Build](#-build)
* [📁 Run](#️-run)
* [🎮 Controls](#-controls)

---

## 🧪 Source Code

```bash
git clone http://gitlab.dobot.com/atom-locomotion/deploy.git
```

## 📦 Dependencies

### ONNX Runtime (from source)

```bash
git clone https://github.com/microsoft/onnxruntime
cd onnxruntime
git checkout v1.19.0
./build.sh --config Release --build_shared_lib --parallel --allow_running_as_root
cd onnxruntime/build/Linux/Release/
sudo make install
```

### DOBOT ATOM SDK

```bash
git clone https://github.com/Dobot-Arm/dobot_atom_sdk.git
cd dobot_atom_sdk
rm -rf build && mkdir build
cd build
cmake ..
make
```

### Boost

```bash
sudo apt-get update
sudo apt-get install libboost-all-dev
```

## 🔧 Build

```bash
cd deploy
mkdir build && cd build
cmake ..
make
```

## 📁 Run

```bash
./atom_deploy
```

---

## 🎮 Controls

Default gamepad mapping (example):

| Operation                  | Combo    | Description                         |
| -------------------------- | -------- | ----------------------------------- |
| Move to default joint position | `A`      | Robot joints return to default position |
| Run model                  | `RB + ↑` | Trigger model inference/task        |

---

