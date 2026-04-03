# nrspath_ws

This workspace contains:

- `nrs_path2` : Surface processing / path planning package
- `nrs_waypoint_generator` : Mesh-based waypoint generation + ROS2 publisher

---

# 1. Environment Setup (Python venv)

Create virtual environment:

```bash
cd ~/nrspath_ws
python3 -m venv env_path
source env_path/bin/activate
```

Upgrade pip:

```bash
python -m pip install --upgrade pip setuptools wheel
```

---

# 2. Python Dependencies

Required for `nrs_waypoint_generator`:

```bash
pip install "numpy<2" scipy trimesh rtree
```

---

# 3. System Dependencies (nrs_path2 + sensors)

## Cyclone DDS

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

---

## RealSense SDK

```bash
sudo mkdir -p /etc/apt/keyrings

curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

sudo apt-get install apt-transport-https

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

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

# 4. Build Workspace

```bash
cd ~/nrspath_ws
colcon build
source install/setup.bash
```

---

# 5. Run

## 5.1 Waypoint Generator Node

```bash
ros2 run nrs_waypoint_generator waypoint_generator   --ros-args   -p mesh:=/home/eunseop/isaac/isaac_save/surface/workpiece_8.stl   -p region_id:=1   -p frame_id:=base_link   -p publish_rate_hz:=1.0
```

---

## 5.2 Output Topic

Published topic:

```
/clicked_point  (geometry_msgs/msg/PointStamped)
```

---

# 6. Notes

- Always activate environment before running:

```bash
source ~/nrspath_ws/env_path/bin/activate
source ~/nrspath_ws/install/setup.bash
```

- `rtree` is required for trimesh raycasting (critical dependency)

- If error occurs:

```
ModuleNotFoundError: No module named 'rtree'
```

→ install:

```bash
pip install rtree
```

---

# 7. Workspace Structure

```
nrspath_ws/
├── src/
│   ├── nrs_path2/
│   └── nrs_waypoint_generator/
├── env_path/
└── README.md
```

---

# 8. Recommended

- Do NOT place virtual environment inside `src/`
- Keep it at workspace root (`~/nrspath_ws/env_path`)
