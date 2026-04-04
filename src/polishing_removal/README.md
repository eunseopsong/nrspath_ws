# polishing_removal

ROS 2 Python package for recording polishing motion/force data from robot topics, computing a polishing-removal heatmap, saving four required PNG outputs, and displaying the heatmap window when recording ends.

This README matches the current split implementation:

- `polishing_removal_node.py`: ROS 2 node, subscriptions, services, recording, processing, `main()`
- `polishing_removal_visualization.py`: all plotting and PNG saving code

---

## 1. What this package does

This package records robot motion and force data while polishing, then visualizes the result.

Workflow:

1. Run the node.
2. Call the `start` service.
3. The node records:
   - position/state from `/ur10skku/currentP`
   - force from `/ur10skku/currentF`
4. Call the `end` service.
5. The node:
   - stops recording
   - computes polishing-removal information
   - saves **exactly 4 PNG files** into `logs/`
   - opens the heatmap with `plt.show()` after saving the PNG files

---

## 2. Topic assumptions

### `/ur10skku/currentP`
Type:

```bash
std_msgs/msg/Float64MultiArray
```

Expected data format:

```text
[x, y, z, wx, wy, wz]
```

Meaning:

- `x, y, z`: position in **mm**
- `wx, wy, wz`: angular-velocity-like 3D signal used for recording and visualization

> Note:
> `wx wy wz` are **not** interpreted as roll-pitch-yaw.
> They are stored and visualized as-is.

### `/ur10skku/currentF`
Type:

```bash
std_msgs/msg/Float64MultiArray
```

Expected data format:

```text
[fx, fy, fz, tx, ty, tz]
```

Meaning:

- `fx, fy, fz`: force in **N**
- `tx, ty, tz`: ignored by this package

Only `fx, fy, fz` are used.

---

## 3. Current package structure

Recommended package layout:

```bash
polishing_removal/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── polishing_removal
├── polishing_removal/
│   ├── __init__.py
│   ├── polishing_removal_node.py
│   └── polishing_removal_visualization.py
└── logs/
```

Inside the Python package:

- `polishing_removal_node.py`
  - ROS services
  - ROS subscriptions
  - timer-based recording
  - removal computation
  - summary/status publication
  - `main()`
- `polishing_removal_visualization.py`
  - heatmap figure generation
  - 4 required PNG exports

---

## 4. Dependencies

### ROS 2 dependencies

Make sure these are available:

- `rclpy`
- `std_msgs`
- `std_srvs`
- `ament_index_python`

### Python dependencies

- `numpy`
- `matplotlib`

Example installation:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-std-msgs \
  ros-humble-std-srvs \
  python3-numpy \
  python3-matplotlib
```

If `ament_index_python` is not already available through your ROS environment, install the corresponding ROS package as needed.

---

## 5. Build

Example workspace:

```bash
~/nrspath_ws
```

Build commands:

```bash
cd ~/nrspath_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select polishing_removal
source install/setup.bash
```

---

## 6. Run

This package is configured so that the node can be started **without long `--ros-args` commands**.

Run:

```bash
ros2 run polishing_removal polishing_removal_node
```

When the node starts, it subscribes to:

- `/ur10skku/currentP`
- `/ur10skku/currentF`

and opens two services:

- `~/start`
- `~/end`

Their full names are typically:

```bash
/polishing_removal_node/start
/polishing_removal_node/end
```

---

## 7. Start / End service calls

### Start recording

```bash
ros2 service call /polishing_removal_node/start std_srvs/srv/Trigger "{}"
```

### End recording and process results

```bash
ros2 service call /polishing_removal_node/end std_srvs/srv/Trigger "{}"
```

What happens when `end` is called:

1. recording stops
2. the node computes polishing-removal data
3. the node saves the 4 required PNG files into `logs/session_YYYYMMDD_HHMMSS/`
4. the heatmap is shown with `plt.show()`

---

## 8. Output location

The node searches for the package root and saves outputs under:

```bash
polishing_removal/logs/
```

A new session folder is created automatically:

```bash
logs/session_YYYYMMDD_HHMMSS/
```

Example:

```bash
polishing_removal/logs/session_20260404_153012/
```

If the workspace source path cannot be found, the node may fall back to a share-directory-based `logs/` path.

---

## 9. Saved output files

Only **4 PNG files** are saved.

### 1) `01_removal_heatmap.png`
Heatmap of polishing removal.

Contains:

- removal heatmap in XY plane
- mean heatmap removal
- standard deviation of heatmap removal
- sample count
- contact sample count

### 2) `02_heatmap_value_vs_time.png`
Time-series plot using the **same physical quantity as the heatmap**.

Important:

- The heatmap itself is built from **cell removal** values.
- This plot shows, at each timestep, the **final heatmap cell value** of the robot's current XY location.
- Therefore, the Y-axis quantity is directly comparable to the heatmap color scale.

### 3) `03_signals_subplot.png`
Single PNG with 9 subplots:

- `x`
- `y`
- `z`
- `wx`
- `wy`
- `wz`
- `fx`
- `fy`
- `fz`

### 4) `04_3d_path_w.png`
3D visualization of the motion path.

Contains:

- 3D XYZ path
- direction arrows generated from `wx wy wz`

---

## 10. Processing logic

### Recording

The node records data with a timer at `recording_rate_hz`.

Default:

```text
50 Hz
```

At each timer tick, if recording is active and both topics have valid data:

- time is stored
- `[x, y, z, wx, wy, wz]` is stored
- `[fx, fy, fz]` is stored

### Velocity computation

Velocity is computed from recorded XYZ positions using numerical differentiation:

- `vx`
- `vy`
- `vz`

Units are assumed to remain in **mm/s** because position is already in **mm**.

### Speed

Default speed mode:

```text
xy
```

So speed is:

```text
sqrt(vx^2 + vy^2)
```

### Tool-axis force projection

The default tool axis is fixed in the base frame.

Default:

```text
+Z
```

The node projects force onto the selected tool axis and keeps the positive contact direction automatically.

### Contact condition

Default thresholds:

- `contact_threshold_N = 0.5`
- `speed_threshold_mm_s = 0.1`

A sample is treated as polishing contact only when both conditions are satisfied.

### Removal model

Removal rate is computed using a Preston-like proportional model:

```text
removal_rate = k_preston * Fn * speed
```

and integrated over time per step.

### Heatmap

The XY workspace is discretized using `cell_mm`.

Default:

```text
cell_mm = 1.0
```

Each sample contributes removal to its corresponding XY cell.

Optional smoothing is applied using a circular kernel.

Default:

```text
pad_radius_mm = 20.0
```

---

## 11. Default parameters

These are the built-in defaults, so `ros2 run polishing_removal polishing_removal_node` works directly.

```text
package_name            = polishing_removal
position_topic          = /ur10skku/currentP
force_topic             = /ur10skku/currentF
recording_rate_hz       = 50.0
tool_axis               = +Z
speed_mode              = xy
contact_threshold_N     = 0.5
speed_threshold_mm_s    = 0.1
cell_mm                 = 1.0
pad_radius_mm           = 20.0
k_preston               = 1.0
w_arrow_stride          = 20
w_arrow_length_mm       = 15.0
```

---

## 12. Optional parameter override

Even though the package is intended to run without extra arguments, parameters can still be overridden if needed.

Example:

```bash
ros2 run polishing_removal polishing_removal_node \
  --ros-args \
  -p tool_axis:=+Z \
  -p cell_mm:=1.0 \
  -p pad_radius_mm:=20.0
```

---

## 13. Useful checks

### Check node info

```bash
ros2 node info /polishing_removal_node
```

### Check service list

```bash
ros2 service list | grep polishing_removal
```

### Check topic types

```bash
ros2 topic info /ur10skku/currentP
ros2 topic info /ur10skku/currentF
```

### Echo one sample

```bash
ros2 topic echo --once /ur10skku/currentP
ros2 topic echo --once /ur10skku/currentF
```

---

## 14. Typical usage example

### Terminal 1

```bash
cd ~/nrspath_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run polishing_removal polishing_removal_node
```

### Terminal 2: start recording

```bash
cd ~/nrspath_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /polishing_removal_node/start std_srvs/srv/Trigger "{}"
```

### Perform robot motion / polishing

Move the robot while the node records data.

### Terminal 2: end recording

```bash
ros2 service call /polishing_removal_node/end std_srvs/srv/Trigger "{}"
```

After that:

- 4 PNG files are saved in `logs/session_.../`
- the heatmap window appears on screen

---

## 15. Behavior of `plt.show()`

After `end` is called and all PNGs are saved, the heatmap is displayed with:

```python
plt.show()
```

This usually means:

- the heatmap window opens after file saving
- the callback may block until the plot window is closed, depending on the matplotlib backend

If no graphical display is available, matplotlib may use a non-interactive backend and the display step may be skipped or behave differently.

---

## 16. Troubleshooting

### Problem: `/ur10skku/currentF` subscription count is 0

Possible causes:

- old node version still running
- package was rebuilt but `source install/setup.bash` was not executed
- the split files were not copied into the package correctly

Recommended fix:

```bash
cd ~/nrspath_ws
colcon build --packages-select polishing_removal
source install/setup.bash
ros2 run polishing_removal polishing_removal_node
```

### Problem: heatmap window does not appear

Possible causes:

- no desktop display
- matplotlib backend is non-interactive
- running over SSH without X forwarding

If plotting on-screen is required, run in an environment with a working GUI display.

### Problem: output path is not where expected

The node tries to find the package source directory first and uses its `logs/` folder.
If that fails, it falls back to a share-directory-based `logs/` path.

---

## 17. Minimal checklist

Before use, verify all of the following:

- `polishing_removal_node.py` and `polishing_removal_visualization.py` are in the package
- package builds successfully
- `/ur10skku/currentP` publishes `[x, y, z, wx, wy, wz]`
- `/ur10skku/currentF` publishes `[fx, fy, fz, tx, ty, tz]`
- the node sees both subscriptions
- `start` and `end` services are available

---

## 18. Summary

This package currently provides:

- ROS 2 recording based on `start` / `end` services
- recording from robot position and force topics
- polishing-removal computation in mm-based coordinates
- saving exactly 4 required PNG visualizations
- showing the heatmap on-screen after saving when `end` is called

