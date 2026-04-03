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

## 1. ROS2 & Build Tools
```bash
sudo apt update

sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions
```

---

## 2. Python (ROS message generation)
```bash
sudo apt install -y \
  python3-lark \
  python3-more-itertools \
  python3-empy
```

---

## 3. RealSense SDK (D435 / L515)
```bash
sudo apt install -y librealsense2-dev librealsense2-utils
```

---

## 4. PCL (Point Cloud Library)
```bash
sudo apt install -y libpcl-dev
```

---

## 5. CGAL
```bash
sudo apt install -y libcgal-dev
```

---

## 6. VTK
```bash
sudo apt install -y libvtk9-dev
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

rm -rf build install log

source /opt/ros/humble/setup.bash

colcon build
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

## Python path issue

```bash
which python3
```

Must be:
```bash
/usr/bin/python3
```

If conda path appears:
```bash
conda deactivate
```

---

## Missing modules

```bash
sudo apt install python3-lark python3-more-itertools python3-empy
```

---

# 🚀 Recommended Workflow

```bash
conda deactivate
source /opt/ros/humble/setup.bash
colcon build
```

---

# 💡 Notes

- Never mix ROS2 build with conda
- Use conda only for ML/RL separately
- Always clean build if environment changes

---

# ✅ Done
