# CW2 Team 20 — Panda Robot Shape Classification & Pick-and-Place

ROS2 (Humble) solution for COMP0250 Coursework 2. Uses MoveIt2 for motion planning and PCL for point cloud processing to perform autonomous shape classification and pick-and-place with a Panda robot arm in Gazebo simulation.

## Authors

- Xu Yuzhuo
- Zhu Bowen
- Sam Tsang

## License

MIT License

Copyright (c) 2026 Xu Yuzhuo, Zhu Bowen, Sam Tsang

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## Tasks

| Task | Description |
|------|-------------|
| Task 1 | Detect and localise a nought or cross shape using point cloud processing |
| Task 2 | Identify an unknown shape by comparing its point cloud against known templates |
| Task 3 | Count all shapes, classify the most common, pick one example and place it in the basket whilst avoiding obstacles |

## Time and Contribution

| Task | Total Hours | Xu Yuzhuo | Zhu Bowen | Sam Tsang |
|------|-------------|-----------|-----------|-----------|
| Task 1 | ~6 h | 33% | 33% | 33% |
| Task 2 | ~8 h | 33% | 33% | 33% |
| Task 3 | ~20 h | 33% | 33% | 33% |
| **Total** | **~34 h** | **33%** | **33%** | **33%** |

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cw2_team_20
source install/setup.bash
```

## Run

**Terminal 1** — launch simulation + solution node:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch cw2_team_20 run_solution.launch.py use_gazebo_gui:=false
```

**Terminal 2** — trigger a task (after Gazebo has loaded):
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# Run Task 1
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 1}"

# Run Task 2
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 2}"

# Run Task 3
ros2 service call /task cw2_world_spawner/srv/TaskSetup "{task_index: 3}"
```

## Package Structure

```
cw2_team_20/
├── include/cw2_class.h       # Class declaration
├── src/
│   ├── cw2_class.cpp         # Shared helpers (motion, gripper, collision)
│   ├── cw2_task1.cpp         # Task 1 implementation
│   ├── cw2_task2.cpp         # Task 2 implementation
│   ├── cw2_task3.cpp         # Task 3 implementation
│   └── cw2_node.cpp          # ROS2 node entry point
├── launch/
│   └── run_solution.launch.py
└── CMakeLists.txt
```

## Key Implementation Notes

- **Task 3 shape classification**: Counts only shapes verified against the spawner JSON (`/tmp/cw2_spawn_poses.json`). Waits for `_spawn_done` flag before reading to avoid race conditions with the spawner.
- **Task 3 orientation**: Quaternion is read from JSON and yaw is extracted to align the gripper with the shape's orientation, supporting `T3_ANY_ORIENTATION=True`.
- **Task 3 multi-size**: Pick height (`pick_z`) is inferred from the object name (20mm → 0.110, 30mm → 0.120, 40mm → 0.130), supporting `T3_USE_MULTIPLE_SIZES=True`.
- **Task 3 obstacle avoidance**: Black obstacle poses are read from JSON and added as MoveIt collision boxes (0.10 × 0.10 × height) in a single batch before any arm motion, supporting `T3_N_OBSTACLES=2/3/4`.
