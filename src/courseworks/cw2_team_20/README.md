# COMP0250 CW2 — Team 20

**UCL COMP0250 Coursework 2 — Pick and Place, Object Detection and Localisation**

Authors: Team 20, UCL  
License: MIT

---

## Overview

This package implements an autonomous pick-and-place system using a Franka Panda robot arm
in Gazebo. It uses MoveIt! for motion planning, an Intel RealSense D435 wrist camera for
perception, PCL for point cloud processing, and OpenCV for shape classification.

### Task 1 — Pick and Place

Given the position of a shape and a basket, the robot:
1. Moves to a pre-grasp pose above the shape.
2. Descends via a Cartesian path, grasps the shape, and lifts it.
3. Moves to a pre-place pose above the basket.
4. Descends via a Cartesian path, releases the shape, and retreats.

The grasp yaw angle is estimated from the principal axis of the shape's point cloud
so that the gripper aligns with the longest axis of the shape.

### Task 2 — Shape Classification

Given two reference shapes and a mystery shape, the robot:
1. Acquires point clouds of all three objects by moving the camera overhead.
2. Classifies each shape as a *nought* (has an interior hole) or *cross* (no hole)
   using a 2-D top-down projection and hole-counting heuristic.
3. Returns the index (1 or 2) of the reference shape that matches the mystery shape.

### Task 3 — Autonomous Scan and Sort

With no prior knowledge of the scene, the robot:
1. Scans the workspace in a grid pattern, collecting point clouds.
2. Segments and classifies all shapes found (noughts and crosses).
3. Identifies the most common shape type and its count.
4. Picks every shape of the most common type and places them in the basket.
5. Returns statistics: total shapes, count of most common, and a shape-type vector.

---

## Dependencies

- ROS 2 Humble
- MoveIt! 2
- `pcl_ros`, `pcl_conversions`, PCL
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- OpenCV 4
- `octomap`, `octomap_msgs`
- `cw2_world_spawner` (part of `comp0250_s26_labs`)
- `rpl_panda_with_rs` (Franka + RealSense simulation)

---

## Setup

1. Clone the UCL lab framework (if not already done):
```bash
cd ~
git clone https://github.com/surgical-vision/comp0250_s26_labs.git
```

2. Clone this repository into the courseworks folder:
```bash
cd ~/comp0250_s26_labs/src/courseworks/
git clone https://github.com/sammytsang/COMP0250_cw2 cw2_team_20
```

---

## Build

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ~/comp0250_s26_labs
colcon build --mixin release --parallel-workers 1
source install/setup.bash
```

---

## Run

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch cw2_team_20 run_solution.launch.py use_gazebo_gui:=true use_rviz:=true
```

Optional arguments:
| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `use_gazebo_gui` | `true` | Show Gazebo window |
| `use_rviz` | `true` | Show RViz window |
| `enable_realsense` | `true` | Enable wrist camera |
| `pointcloud_topic` | `/r200/camera/depth_registered/points` | Point cloud topic |
| `pointcloud_qos_reliable` | `true` | Use reliable QoS for point cloud |
| `launch_delay` | `5.0` | Seconds to wait before spawning world |

---

## Triggering Tasks

After launching, trigger each task via the world-spawner service:

```bash
# Task 1
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 1}"

# Task 2
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 2}"

# Task 3
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 3}"
```

---

## Fixing the World Spawner Crash (Gazebo Entity State Race Condition)

When calling `/task` immediately after spawning objects, the world spawner may crash with:

```
AttributeError: 'NoneType' object has no attribute 'pose'
```

This happens because Gazebo hasn't fully registered freshly-spawned entities when
`get_model_state_by_name` / `get_model_state_via_gz` are first called.

### Apply the fix

Run the provided patch script **once** after cloning this repo:

```bash
bash ~/comp0250_s26_labs/src/courseworks/cw2_team_20/scripts/fix_gazebo_state_retry.sh
```

The script patches **both** the installed copy and the source copy of
`coursework_world_spawner.py` to:

1. **`get_model_state_by_name`** — waits up to 10 s for the
   `/gazebo/get_entity_state` service and retries the call up to 10 times
   (1 s between attempts) before falling back to the CLI.
2. **`get_model_state_via_gz`** — retries `gz model -m <name> -i` up to 5
   times (1 s between attempts) before returning `None`.

No `colcon build` is required for `cw2_world_spawner` after running the script
(it is a pure-Python package).

---

## Time Contribution

| Team Member | Contribution |
|---|---|
| Member 1 | — |
| Member 2 | — |
| Member 3 | — |
| Member 4 | — |
