# SLAM & RL Resource Manager Workspace (ROS 2 Jazzy)

A workspace for SLAM, reactive exploration, and RL-based resource management on TurtleBot3.

## Components

- **`tb3_reactive_explorer`**: Obstacle avoidance and random wandering (`/scan` → `/cmd_vel`).
- **`mini_mapper`**: Simple occupancy-grid mapper (`/odom` + `/scan` → `/mini_map`).
- **`rl_resource_manager`**: Q-learning agent managing system resources (nice values) to profiling SLAM performance under load.
- **`experiment_metrics.py`**: Benchmarking script for measuring jitter, stamp age, and resource usage.

## Setup & Build

```bash
# 1. Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Build the workspace
cd ~/slam_rl_ws
colcon build --symlink-install

# 3. Source the workspace
source ~/slam_rl_ws/install/setup.bash
```

## Running the System

Open multiple terminals to run the components.

### Terminal 1: Simulation (Gazebo)
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Terminal 2: SLAM (SLAM Toolbox)
Basic launch:
```bash
ros2 launch slam_toolbox online_async_launch.py
```
Or with custom simulation time and parameters:
```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=$(pwd)/mapper_params_online_async.yaml
```
*(Note: Run this from the workspace root where the params file is located)*

### Terminal 3: Explorer Agent
Runs the reactive exploration node to fetch sensor data and drive the robot.
```bash
ros2 run tb3_reactive_explorer reactive_explorer --ros-args \
  -p use_sim_time:=true \
  -p forward_speed:=0.15 \
  -p turn_speed:=0.5 \
  -p avoid_front:=0.5 \
  -p wander_turn_every_s:=10.0
```

### Terminal 4: Resource Manager & CPU Load (RL)
Run the resource manager to adaptively control process priorities:
```bash
ros2 run rl_resource_manager rl_resource_manager
```

Run a background CPU stress node ("Hog") to induce load:
```bash
ros2 run rl_resource_manager cpu_hog --ros-args -p load:=0.8
```

### Terminal 5: Visualization
```bash
ros2 run rviz2 rviz2
```
*Tip: In RViz, set Fixed Frame to `map` and add the `/map` and `/scan` topics.*

---

## Running Experiments

Use the provided script to run a controlled experiment measuring SLAM performance under increasing load.

```bash
cd ~/slam_rl_ws
python3 experiment_metrics.py \
  --loads 0.0,0.5,0.8,0.95 \
  --start-after 5 \
  --step 30 \
  --duration 150 \
  --outdir metrics_out
```

### Saving the Map
To save the generated map to disk:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
