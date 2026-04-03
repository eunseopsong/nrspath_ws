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

## 2. Python (ROS message generation)
```bash
sudo apt install -y \
  python3-lark \
  python3-more-itertools \
  python3-empy
```

## 3. Cyclone DDS
```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## 4. RealSense SDK
```bash
sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | \
sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

sudo apt-get install apt-transport-https

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

sudo apt-get update

sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# optional
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```

## 5. PCL
```bash
sudo apt-get update
sudo apt-get install pcl-tools
sudo apt-get install -y libpcl-dev
```

## 6. CGAL
```bash
sudo apt-get install libcgal-dev

wget https://github.com/CGAL/cgal/releases/download/v5.6.1/CGAL-5.6.1.tar.xz
tar xf CGAL-5.6.1.tar.xz
cd CGAL-5.6.1

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install
```

## 7. VTK
```bash
sudo apt-get install libvtk7-dev
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
rm -rf ~/nrspath_ws/env_path
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

# ▶️ Package Run Order

Open a new terminal for each long-running node when needed, and source ROS 2 + workspace first:

```bash
source /opt/ros/humble/setup.bash
source ~/nrspath_ws/install/setup.bash
```

## 1. Launch path planning
```bash
ros2 launch nrs_path2 path_planning.launch.py
```

## 2. Run waypoint generation node

Choose the command according to the target surface mesh.

### Example: workpiece_1
```bash
ros2 run nrs_waypoint_generator waypoint_generator \
  --ros-args \
  -p mesh:=/home/eunseop/isaac/isaac_save/surface/workpiece_1.stl \
  -p region_id:=1 \
  -p frame_id:=base_link \
  -p publish_rate_hz:=1.0
```

### Example: workpiece_2
```bash
ros2 run nrs_waypoint_generator waypoint_generator \
  --ros-args \
  -p mesh:=/home/eunseop/isaac/isaac_save/surface/workpiece_2.stl \
  -p region_id:=1 \
  -p frame_id:=base_link \
  -p publish_rate_hz:=1.0
```

### Example: workpiece_8
```bash
ros2 run nrs_waypoint_generator waypoint_generator \
  --ros-args \
  -p mesh:=/home/eunseop/isaac/isaac_save/surface/workpiece_8.stl \
  -p region_id:=1 \
  -p frame_id:=base_link \
  -p publish_rate_hz:=1.0
```

Change `workpiece_X.stl` to the surface you want to use.
Also change `region_id` to one of `1, 2, 3, 4` depending on the region to publish.

## 3. Generate path
Use either straight path or spline path:

```bash
ros2 service call /straight std_srvs/srv/Empty "{}"
```

or

```bash
ros2 service call /spline std_srvs/srv/Empty "{}"
```

## 4. Run interpolation
```bash
ros2 service call /interpolation std_srvs/srv/Empty "{}"
```

---

# ▶️ Generic Run

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
