# nrspath_ws

This workspace contains:

- `nrs_path2`: Surface processing / path planning package
- `nrs_waypoint_generator`: Mesh-based waypoint generation and ROS 2 publisher

---

# 1. Remove Existing Python Environment

If you previously created a virtual environment inside the workspace, remove it first.

```bash
rm -rf ~/nrspath_ws/env_path
rm -rf ~/nrspath_ws/src/env_path
```

---

# 2. Create Conda Environment

Create and activate a new Conda environment named `env_path`.

```bash
conda create -n env_path python=3.10 -y
conda activate env_path
```

It is recommended to install build tools first:

```bash
pip install --upgrade pip setuptools==65.5.0 wheel
```

---

# 3. Python Dependencies

Required for `nrs_waypoint_generator`:

```bash
pip install "numpy<2" scipy trimesh rtree catkin_pkg pyyaml jinja2 typeguard
```

If `rtree` installation fails, install the system dependency first:

```bash
sudo apt-get update
sudo apt-get install -y libspatialindex-dev
pip install rtree
```

---

# 4. System Dependencies

## Cyclone DDS

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

---

## RealSense SDK

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

---

## PCL

```bash
sudo apt-get update
sudo apt-get install pcl-tools
```

---

## CGAL

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

---

## VTK

```bash
sudo apt-get install libvtk7-dev
```

---

# 5. Build Workspace

Before building, activate the Conda environment and source ROS 2:

```bash
conda activate env_path
source /opt/ros/humble/setup.bash
```

Then build:

```bash
cd ~/nrspath_ws
rm -rf build install log
colcon build
source install/setup.bash
```

---

# 6. Run

## 6.1 Waypoint Generator Node

```bash
conda activate env_path
source /opt/ros/humble/setup.bash
source ~/nrspath_ws/install/setup.bash

ros2 run nrs_waypoint_generator waypoint_generator \
  --ros-args \
  -p mesh:=/home/eunseop/isaac/isaac_save/surface/workpiece_8.stl \
  -p region_id:=1 \
  -p frame_id:=base_link \
  -p publish_rate_hz:=1.0
```

---

## 6.2 Output Topic

Published topic:

```text
/clicked_point  (geometry_msgs/msg/PointStamped)
```

---

# 7. Recommended Shell Setup

To avoid Python package conflicts during build:

```bash
conda activate env_path
export PYTHONNOUSERSITE=1
unset PYTHONPATH
source /opt/ros/humble/setup.bash
```

Then build or run ROS 2 commands.

---

# 8. Workspace Structure

```text
nrspath_ws/
├── src/
│   ├── nrs_path2/
│   ├── nrs_waypoint/
│   └── nrs_waypoint_generator/
└── README.md
```

---

# 9. Notes

- Do not place Python environments inside `src/`
- Use the Conda environment from outside the ROS package directories
- If `colcon` scans unwanted folders, make sure no Conda or Python environment exists inside `src/`
