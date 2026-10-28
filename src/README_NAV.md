# nav_sim - Nav2 Navigation for the slam_sim Robot (ROS 2 Humble)

Autonomous navigation for the differential-drive robot (2D lidar + depth camera) in the four-room Gazebo world from the `slam_sim` package. Click a goal in RViz2 and the robot plans a path and drives there, avoiding walls and obstacles.

- Global planner: NavFn (Dijkstra, optional A*)
- Local planner / controller: Regulated Pure Pursuit (default) or DWB (alternative)
- Localization: AMCL on a pre-made map (default), or slam_toolbox (slam:=true)
- Costmaps: static layer + lidar obstacle layer + inflation layer
- Simulator: Gazebo Classic
- Visualization: RViz2 with path lines, costmaps and goal tools

Note: this package depends on `slam_sim` (robot model, Gazebo world, SLAM launch files). Both packages live in the same workspace.

---

## Contents

1. How it works
2. Requirements
3. Package contents
4. Install and build
5. Running
6. Sending goals
7. Reading the RViz2 display
8. Configuration
9. Switching the local planner
10. Using your own map
11. Topics and frames
12. Verifying the system
13. Troubleshooting
14. Known limitations

---

## 1. How it works

```
RViz2 "2D Goal Pose"  ->  /goal_pose
                              |
                              v
                        bt_navigator  (behaviour tree)
                         |         |
              goal       |         |  stuck / failed
                         v         v
              planner_server     behavior_server
              (NavFn)            (spin, backup, wait)
                   |
                   v
              /plan  (global path, green line)
                   |
                   v
              controller_server  (Pure Pursuit or DWB)
                   |
                   v
              /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> Gazebo wheels

map_server -> /map -> AMCL -> map->odom transform
Gazebo diff-drive -> odom->base_footprint transform, /odom
Gazebo lidar -> /scan -> AMCL and costmaps
```

1. Localization. `map_server` loads `rooms_map.yaml`. AMCL matches the lidar scan against this map and publishes the `map -> odom` transform. The Gazebo diff-drive plugin publishes `odom -> base_footprint`. Together they give the robot's pose in the map.
2. Costmaps. The global costmap (whole map) and the local costmap (a 3 x 3 m window around the robot) mark walls and obstacles and inflate them by the robot size, so planned paths keep a safe distance.
3. Global planning. NavFn computes a collision-free path from the robot to the goal over the global costmap. It is drawn as the green line.
4. Local control. The controller follows the global path, reacting to what the lidar sees in the local costmap, and outputs velocity commands.
5. Velocity smoothing. `velocity_smoother` limits acceleration so the robot starts and stops gently. Its output is `/cmd_vel`, which drives the Gazebo wheels.
6. Recovery. If the robot gets stuck, the behaviour tree clears the costmaps and runs recovery behaviours (spin, back up, wait) before retrying or aborting.

---

## 2. Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (gazebo_ros_pkgs)
- The `slam_sim` package in the same workspace

---

## 3. Package contents

```
nav_sim/
|-- CMakeLists.txt
|-- package.xml
|-- config/
|   |-- nav2_params.yaml        Nav2 parameters, Pure Pursuit local planner (default)
|   `-- nav2_params_dwb.yaml    Same, with the DWB local planner
|-- launch/
|   `-- nav2_sim.launch.py      Gazebo + robot + Nav2 + RViz2
|-- maps/
|   |-- rooms_map.pgm           Occupancy grid of the rooms world
|   `-- rooms_map.yaml          Map metadata (resolution 0.05 m, origin -6.1, -6.1)
|-- rviz/
|   `-- nav.rviz                RViz2 layout with path lines and costmaps
|-- scripts/
|   `-- send_goals.py           Automatic tour of the four rooms
`-- tools/
    `-- generate_map.py         Regenerates the map from the world geometry
```

---

## 4. Install and build

```bash
# Nav2 and tools
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander ros-humble-nav2-regulated-pure-pursuit-controller

# Build (workspace containing slam_sim and nav_sim)
cd ~/slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in every new terminal.

---

## 5. Running

### 5.1 Navigation on the known map (default)

```bash
ros2 launch nav_sim nav2_sim.launch.py
```

Startup sequence:

- 0 s: Gazebo, robot spawn and RViz2 start
- 12 s: Nav2 starts (map server, AMCL, planner, controller, behaviours)
- 22 s: the start pose (-3.0, 1.5) is sent to AMCL automatically
- about 30 s: the system is ready, send goals

The robot spawns at (-3.0, 1.5) facing east in the NW room, which matches the AMCL initial pose. Wait about 30 seconds before sending the first goal.

### 5.2 Navigation while mapping (SLAM mode)

```bash
ros2 launch nav_sim nav2_sim.launch.py slam:=true
```

`slam_toolbox` builds the map as the robot moves, and Nav2 plans on that live map instead of the saved one. Early goals may fail until the map covers the area. Save the result with:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/rooms_map
```

### 5.3 Launch arguments

- `slam` (default `false`): `true` uses slam_toolbox instead of AMCL + saved map
- `map` (default `maps/rooms_map.yaml`): full path to a map YAML, ignored when `slam:=true`
- `params_file` (default `config/nav2_params.yaml`): Nav2 parameter file
- `rviz` (default `true`): open RViz2
- `gui` (default `true`): show the Gazebo window, `false` runs headless and is faster

Examples:

```bash
ros2 launch nav_sim nav2_sim.launch.py gui:=false
ros2 launch nav_sim nav2_sim.launch.py map:=$HOME/rooms_map.yaml
ros2 launch nav_sim nav2_sim.launch.py params_file:=$HOME/slam_ws/src/nav_sim/config/nav2_params_dwb.yaml
```

---

## 6. Sending goals

### 6.1 From RViz2

1. Click "2D Goal Pose" in the toolbar.
2. Click the target point on the map and drag to set the final heading.
3. A green path appears and the robot starts moving.

If AMCL is not localized (the robot model does not match the scan), click "2D Pose Estimate" first and set the robot's true position.

### 6.2 Automatic room tour

```bash
ros2 run nav_sim send_goals.py          # AMCL mode
ros2 run nav_sim send_goals.py --slam   # SLAM mode
```

The robot visits these goals in order, passing through the doorways (x, y in metres):

1. NE room: (3.0, 2.0)
2. SE room: (2.0, -2.0)
3. SW room: (-3.0, -2.0)
4. NW room (start): (-3.0, 1.5)

Edit the `GOALS` list in `scripts/send_goals.py` to change the route.

### 6.3 From the command line

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}"
```

---

## 7. Reading the RViz2 display

- Green line (`/plan`): global path from the robot to the goal
- Red line (`/local_plan`): short-term path from the local planner (DWB only)
- Grey/black map (`/map`): static occupancy map
- Coloured overlay, wide (`/global_costmap/costmap`): global costmap with inflation
- Coloured overlay, 3 x 3 m (`/local_costmap/costmap`): local costmap around the robot
- Red dots (`/scan`): lidar hits
- Magenta outline (`/local_costmap/published_footprint`): robot footprint used for collision checks

Costmap colours: cyan is inflated cost, purple/magenta is inscribed (the robot would touch), red is a lethal obstacle.

If no green line appears after setting a goal, the global planner is not producing a path (see Troubleshooting).

---

## 8. Configuration

All Nav2 settings are in `config/nav2_params.yaml`. The main ones:

Planner
- `planner_server.GridBased.plugin`: `nav2_navfn_planner/NavfnPlanner`, the global planner
- `planner_server.GridBased.use_astar`: `False`, set `True` to switch Dijkstra to A*
- `planner_server.GridBased.tolerance`: `0.5`, how far from the goal a path may end

Controller
- `controller_server.controller_frequency`: `10.0`, control loop rate in Hz
- `controller_server.failure_tolerance`: `1.0`, seconds of controller failure allowed before aborting
- `progress_checker.required_movement_radius`: `0.2`, the robot must move this far...
- `progress_checker.movement_time_allowance`: `20.0`, ...within this many seconds
- `general_goal_checker.xy_goal_tolerance`: `0.15`, position tolerance at the goal in metres
- `general_goal_checker.yaw_goal_tolerance`: `0.25`, heading tolerance at the goal in radians
- `FollowPath.desired_linear_vel`: `0.25`, cruise speed in m/s (Pure Pursuit)
- `FollowPath.lookahead_dist`: `0.6`, how far ahead on the path the robot aims, in metres

Costmaps
- `robot_radius` (local and global costmap): `0.25`, robot circle radius used for collision
- `inflation_layer.inflation_radius`: `0.55`, safety margin around obstacles in metres
- `inflation_layer.cost_scaling_factor`: `3.0`, how quickly cost drops with distance

Speed limits and localization
- `velocity_smoother.max_velocity`: `[0.25, 0.0, 0.8]`, max forward speed (m/s) and turn rate (rad/s)
- `amcl.initial_pose`: (-3.0, 1.5, 0), start pose given to AMCL

The doorways are 1.4 m wide. With the 0.25 m robot radius there is about 0.45 m of clear space on each side, so the robot fits through comfortably. If it hesitates at doors, lower `inflation_radius` to `0.45`.

---

## 9. Switching the local planner

Regulated Pure Pursuit (default) follows the global path directly and slows for curves. It needs little tuning and suits simple differential-drive robots.

DWB samples many candidate trajectories each cycle and scores them with critics. It reacts to obstacles more freely but needs more tuning and more CPU.

```bash
# Pure Pursuit (default)
ros2 launch nav_sim nav2_sim.launch.py

# DWB
ros2 launch nav_sim nav2_sim.launch.py \
  params_file:=$HOME/slam_ws/src/nav_sim/config/nav2_params_dwb.yaml
```

The two files differ only in the `FollowPath` block under `controller_server`.

---

## 10. Using your own map

1. Create a map with `slam_sim` (see its README) and save it:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/rooms_map
```

2. Launch with it:

```bash
ros2 launch nav_sim nav2_sim.launch.py map:=$HOME/rooms_map.yaml
```

A map saved from SLAM has its origin at the robot's start position, not at the world origin. The AMCL start pose in `nav2_params.yaml` (and the automatic initial pose in the launch file) assume the provided map, where the robot starts at (-3.0, 1.5). With your own map, set the start pose with "2D Pose Estimate" in RViz2.

### Regenerating the provided map

If you edit `slam_sim/worlds/rooms.world`, update the wall and obstacle lists in `tools/generate_map.py` to match, then run:

```bash
cd ~/slam_ws/src/nav_sim
python3 tools/generate_map.py
```

---

## 11. Topics and frames

Topics
- `/goal_pose` (geometry_msgs/PoseStamped): from RViz2
- `/plan` (nav_msgs/Path): from planner_server
- `/local_plan` (nav_msgs/Path): from the DWB controller
- `/cmd_vel_nav` (geometry_msgs/Twist): from controller_server
- `/cmd_vel` (geometry_msgs/Twist): from velocity_smoother to Gazebo
- `/scan` (sensor_msgs/LaserScan): from the Gazebo lidar
- `/odom` (nav_msgs/Odometry): from the Gazebo diff-drive plugin
- `/map` (nav_msgs/OccupancyGrid): from map_server (or slam_toolbox)
- `/global_costmap/costmap`, `/local_costmap/costmap` (nav_msgs/OccupancyGrid): from the costmap nodes
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): from RViz2 or the launch file

Transform tree

```
map -> odom -> base_footprint -> base_link -> lidar_link
                                           -> camera_link -> camera_optical_link
                                           -> left_wheel / right_wheel
```

`map -> odom` comes from AMCL (or slam_toolbox). `odom -> base_footprint` comes from Gazebo.

---

## 12. Verifying the system

Run these in a new terminal (source ROS and the workspace first).

```bash
# All Nav2 nodes should report "active"
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator

# Localization transform should print continuously
ros2 run tf2_ros tf2_echo map odom

# Sensors
ros2 topic hz /scan
ros2 topic hz /odom

# Test the global planner on its own
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
"{goal: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}, use_start: false}"

# Watch what the controller sends to the wheels while driving
ros2 topic echo /cmd_vel
```

Save the launch output to a file to inspect warnings:

```bash
ros2 launch nav_sim nav2_sim.launch.py 2>&1 | tee ~/nav.log
grep -E "WARN|ERROR" ~/nav.log | tail -30
```

---

## 13. Troubleshooting

Goal is ignored
- Cause: Nav2 is not active yet, or localization is missing.
- Fix: wait 30 s and check `ros2 lifecycle get /bt_navigator`. Click "2D Pose Estimate" if localization is missing.

No green path after setting a goal
- Cause: the global planner failed.
- Fix: run the `compute_path_to_pose` test in section 12 and read the planner error in the launch output.

Costmap not shown or incomplete
- Cause: `map -> odom` is missing, so AMCL has no pose.
- Fix: check `ros2 run tf2_ros tf2_echo map odom`, then set the pose with "2D Pose Estimate".

Robot model does not line up with the lidar scan in RViz2
- Cause: localization is wrong.
- Fix: re-set the pose with "2D Pose Estimate".

"Message Filter dropping message" or "queue is full"
- Cause: the simulation is too slow.
- Fix: check the Real Time Factor in Gazebo, use `gui:=false`, and close other heavy programs.

Robot moves a little, rotates, then stops
- Cause: the controller failed and recovery ran.
- Fix: read the `controller_server` warnings in `~/nav.log` ("Failed to make progress", "No valid trajectories", "collision ahead"). Try the other local planner (section 9).

Robot refuses to enter a doorway
- Cause: inflation is too large.
- Fix: lower `inflation_radius` to `0.45`.

Robot stops very close to the goal
- Fix: increase `xy_goal_tolerance` or `yaw_goal_tolerance`.

"Package 'nav_sim' not found"
- Fix: run `source ~/slam_ws/install/setup.bash`.

Old Gazebo processes block restart
- Fix: `killall -9 gzserver gzclient`

RViz2 panel or display fails to load
- Fix: confirm `ros-humble-navigation2` is installed. Remove `~/.rviz2` if a stale config interferes.

---

## 14. Known limitations

- The costmaps use only the 2D lidar. The depth camera is simulated by `slam_sim` but is not used for navigation.
- Gazebo lidar and camera run on the CPU, so a slow computer or VM may give a low Real Time Factor. This is the most common cause of delayed or erratic behaviour.
- The provided parameters are written for Nav2 on ROS 2 Humble, whose `navigation_launch.py` includes a velocity smoother. Other ROS distributions may need parameter changes.
- The default AMCL start pose and the automatic initial-pose message match the provided map and spawn position only.
