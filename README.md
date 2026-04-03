# NRS Path Workspace (ROS2 Humble - System Python Only)

This repository contains ROS2 packages for waypoint generation and path execution.

---

# ⚠️ IMPORTANT

This project MUST use **system Python (NOT conda)**.

DO NOT activate any conda environment when building or running ROS2.

---

# 🖥️ Environment

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10 (system)

---

# 📦 Dependencies

Install all required dependencies using apt:

```bash
sudo apt update

# ROS2 dependencies
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions

# Python dependencies (ROS message generation)
sudo apt install -y \
  python3-lark \
  python3-more-itertools \
  python3-empy
```

---

# 📁 Workspace Setup

```bash
cd ~
mkdir -p nrspath_ws/src
cd nrspath_ws/src
```

Clone your repositories:

```bash
git clone <your_repo_url>
```

---

# 🚫 Remove Wrong Virtual Environments

If `env_path` exists inside `src`, remove it:

```bash
rm -rf ~/nrspath_ws/src/env_path
```

---

# 🔧 Build

```bash
cd ~/nrspath_ws

# Clean previous build
rm -rf build install log

# Source ROS2
source /opt/ros/humble/setup.bash

# Build
colcon build
```

(Optional alias)
```bash
alias cb='colcon build'
```

---

# ▶️ Run

```bash
cd ~/nrspath_ws
source install/setup.bash

ros2 run <package_name> <node_name>
```

---

# 🔍 Troubleshooting

## 1. Python path issue

Check python:

```bash
which python3
```

Must be:

```bash
/usr/bin/python3
```

If you see:

```bash
/anaconda3/...
```

👉 You are using conda → deactivate it:

```bash
conda deactivate
```

---

## 2. Missing modules

If you see errors like:

```
ModuleNotFoundError: No module named 'lark'
ModuleNotFoundError: No module named 'more_itertools'
```

Install:

```bash
sudo apt install python3-lark python3-more-itertools
```

---

## 3. rosidl / empy error

Error:

```
module 'em' has no attribute 'BUFFERED_OPT'
```

Fix:

```bash
sudo apt install python3-empy
```

---

# 🚀 Recommended Workflow

```bash
# ROS work
conda deactivate
source /opt/ros/humble/setup.bash

# Then build/run
colcon build
```

---

# 💡 Notes

- Never mix ROS2 build with conda environments
- Use conda only for ML / RL projects separately
- Always clean build if environment changes

---

# ✅ Done

If you follow this README step-by-step, the workspace should build and run without issues.
