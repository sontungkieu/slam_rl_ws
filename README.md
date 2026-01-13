# slam_rl_ws (ROS2 Jazzy) — minimal SLAM + reactive exploration + RL resource management

Packages:
- tb3_reactive_explorer: `/scan` -> `/cmd_vel` exploration
- mini_mapper: ray casting + log-odds demo mapper, publishes `/mini_map`
- rl_resource_manager: RL adjusts OS nice values to protect SLAM, includes `cpu_hog` to generate contention

## Build
```bash
mkdir -p ~/slam_rl_ws/src
# copy packages from this zip into ~/slam_rl_ws/src
cd ~/slam_rl_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source slam_rl_ws/install/setup.bash
```

## Run (TurtleBot3 Gazebo typical)
Terminal 1:
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Terminal 2:
```bash
ros2 launch slam_toolbox online_async_launch.py
```

Terminal 3:
```bash
ros2 run tb3_reactive_explorer reactive_explorer
ros2 run tb3_reactive_explorer reactive_explorer --ros-args \
  -p use_sim_time:=true \
  -p forward_speed:=0.10 \
  -p turn_speed:=0.45 \
  -p avoid_front:=0.65 \
  -p avoid_side:=0.50 \
  -p wander_turn_every_s:=12.0 \
  -p wander_turn_duration_s:=0.8
```

ros2 run rviz2 rviz2

(Optional) Terminal 4:
```bash
ros2 run mini_mapper mini_mapper
```

(Optional) Terminal 5:
```bash
ros2 run rl_resource_manager cpu_hog --ros-args -p load:=0.9
ros2 run rl_resource_manager rl_resource_manager
```

## Save map (from `/map`)
```bash
mkdir -p ~/robot_maps
ros2 run nav2_map_server map_saver_cli -f ~/robot_maps/my_map
```

## RViz tips
- For raw scan test: Fixed Frame = `odom`, add `/scan` LaserScan.
- For SLAM map: Fixed Frame = `map`, add `/map` Map display.
