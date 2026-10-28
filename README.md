# ROS 2 Humble SLAM and Navigation Simulation (slam_sim + nav_sim)

A complete ROS 2 Humble simulation of a differential-drive robot with a 2D lidar and a depth camera, running in a four-room Gazebo world. The workspace has two packages that build on each other:

- `slam_sim` builds a map with slam_toolbox while you drive the robot.
- `nav_sim` uses that map (or a live map) to navigate autonomously with Nav2, using the NavFn global planner and a Pure Pursuit or DWB local planner.

Everything starts from single launch files, and RViz2 is preconfigured to show the map, the sensors, the costmaps and the planned path.

---

## Contents

1. Overview
2. Repository layout
3. Requirements and installation
4. Build
5. Part A - SLAM (slam_sim)
6. Part B - Navigation (nav_sim)
7. The robot
8. The world
9. Theory in brief
10. Topics and transform tree
11. Useful commands
12. Troubleshooting
13. Known limitations
14. Possible extensions

---

## 1. Overview

What you get:

- A differential-drive robot model (Xacro) with a 360-degree lidar and a depth camera
- A 12 x 12 m square Gazebo world with four rooms, 1.4 m doorways and box/cylinder obstacles
- Online SLAM with slam_toolbox, with loop closure
- A ready-made occupancy map of the world for navigation
- Nav2 navigation with AMCL localization, NavFn global planning and Regulated Pure Pursuit (default) or DWB local planning
- Optional navigation while mapping (SLAM mode)
- RViz2 layouts, a keyboard teleop workflow, an automatic room-tour script and map-saving instructions

The two stages of a typical session:

```
Stage 1: MAP                          Stage 2: NAVIGATE
drive the robot with the keyboard     click a goal in RViz2
slam_toolbox builds /map              AMCL localizes on the saved map
save the map                          NavFn plans, Pure Pursuit drives
```

---

## 2. Repository layout

```
slam_ws/
|-- README.md                     this file
`-- src/
    |-- slam_sim/                 robot, world and SLAM
    |   |-- CMakeLists.txt
    |   |-- package.xml
    |   |-- config/
    |   |   `-- slam_toolbox.yaml     slam_toolbox parameters
    |   |-- launch/
    |   |   |-- sim.launch.py         Gazebo + robot + RViz2
    |   |   |-- slam.launch.py        slam_toolbox node
    |   |   `-- bringup.launch.py     sim + SLAM together
    |   |-- rviz/
    |   |   `-- slam.rviz             RViz2 layout for mapping
    |   |-- urdf/
    |   |   `-- robot.urdf.xacro      robot model, sensors, Gazebo plugins
    |   `-- worlds/
    |       `-- rooms.world           four-room Gazebo world
    `-- nav_sim/                  Nav2 navigation
        |-- CMakeLists.txt
        |-- package.xml
        |-- README.md                 nav_sim-specific documentation
        |-- config/
        |   |-- nav2_params.yaml      Nav2 parameters, Pure Pursuit (default)
        |   `-- nav2_params_dwb.yaml  Nav2 parameters, DWB local planner
        |-- launch/
        |   `-- nav2_sim.launch.py    Gazebo + robot + Nav2 + RViz2
        |-- maps/
        |   |-- rooms_map.pgm         occupancy map of the world
        |   `-- rooms_map.yaml        map metadata
        |-- rviz/
        |   `-- nav.rviz              RViz2 layout with path lines and costmaps
        |-- scripts/
        |   `-- send_goals.py         automatic tour of the four rooms
        `-- tools/
            `-- generate_map.py       regenerates the map from the world
```

`nav_sim` reuses the robot, the world and the SLAM launch files from `slam_sim`, so the two packages must be built together.

---

## 3. Requirements and installation

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- A computer with at least 4 CPU cores is recommended. The lidar and depth camera are simulated on the CPU.

Install the dependencies:

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-nav2-map-server \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander \
  ros-humble-nav2-regulated-pure-pursuit-controller
```

---

## 4. Build

```bash
cd ~/slam_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Source `install/setup.bash` in every new terminal. The `--symlink-install` flag lets you edit launch files, parameters, URDF and RViz configs without rebuilding.

If you replace the workspace with a new copy, delete the old build first:

```bash
rm -rf build install log
```

---

## 5. Part A - SLAM (slam_sim)

### 5.1 Start the simulation with SLAM

```bash
ros2 launch slam_sim bringup.launch.py
```

This opens Gazebo (robot in the NW room), RViz2 and the slam_toolbox node. SLAM starts 8 seconds after the simulation so that Gazebo, TF and the lidar are ready.

Launch options:

```bash
ros2 launch slam_sim bringup.launch.py gui:=false    # no Gazebo window, faster
ros2 launch slam_sim bringup.launch.py rviz:=false   # no RViz2
ros2 launch slam_sim sim.launch.py                   # simulation only
ros2 launch slam_sim slam.launch.py                  # SLAM node only
```

### 5.2 Drive the robot

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.8
```

Keep that terminal focused while you press keys:

- `i` forward, `,` backward
- `j` rotate left, `l` rotate right
- `u` and `o` move forward while turning
- `k` stop
- `q` and `z` increase or decrease all speeds by 10 percent

### 5.3 Build a good map

1. Wait about 10 seconds until the map appears in RViz2.
2. Drive slowly. Fast turns degrade scan matching.
3. Explore the NW room first, then pass through the doorways into the other rooms.
4. Return to a place you have already visited. The map snaps into alignment when slam_toolbox detects a loop closure.
5. When the map is complete, save it.

### 5.4 Save the map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/rooms_map
```

This creates `rooms_map.pgm` (the image) and `rooms_map.yaml` (resolution and origin).

To save the pose graph so you can continue mapping later:

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/$USER/rooms_graph'}"
```

### 5.5 slam_toolbox settings

Key parameters in `slam_sim/config/slam_toolbox.yaml`:

- `mode: mapping` builds a new map (use `localization` to localize on a saved graph)
- `resolution: 0.05` map cell size in metres
- `max_laser_range: 10.0` ignore beams longer than this
- `map_update_interval: 2.0` seconds between map publications
- `minimum_travel_distance` and `minimum_travel_heading: 0.2` how far the robot must move or turn before a new scan is added
- `do_loop_closing: true` enables loop closure
- `loop_search_maximum_distance: 3.0` how far to look for loop-closure candidates
- `solver_plugin: solver_plugins::CeresSolver` the optimizer that corrects the pose graph

---

## 6. Part B - Navigation (nav_sim)

### 6.1 Navigate on the known map (default)

```bash
ros2 launch nav_sim nav2_sim.launch.py
```

Startup sequence:

- 0 s: Gazebo, robot spawn and RViz2 start
- 12 s: Nav2 starts (map server, AMCL, planner, controller, behaviours)
- 22 s: the start pose (-3.0, 1.5) is sent to AMCL automatically
- about 30 s: ready, send goals

The robot spawns at (-3.0, 1.5) facing east in the NW room, which matches the AMCL initial pose. Wait about 30 seconds before sending the first goal.

### 6.2 Send goals

From RViz2:

1. Click "2D Goal Pose" in the toolbar.
2. Click the target point on the map and drag to set the final heading.
3. A green line shows the planned path and the robot drives along it.

If the robot model does not line up with the lidar scan, localization is off. Click "2D Pose Estimate" and set the robot's true position.

Automatic tour of the four rooms:

```bash
ros2 run nav_sim send_goals.py          # AMCL mode
ros2 run nav_sim send_goals.py --slam   # SLAM mode
```

The goals, in order, are the NE room (3.0, 2.0), the SE room (2.0, -2.0), the SW room (-3.0, -2.0) and back to the NW start (-3.0, 1.5).

From the command line:

```bash
ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}"
```

### 6.3 Navigate while mapping (SLAM mode)

```bash
ros2 launch nav_sim nav2_sim.launch.py slam:=true
```

slam_toolbox builds the map live and Nav2 plans on it instead of the saved map. Early goals may fail until the map covers the area.

### 6.4 Launch arguments

- `slam` (default false): true uses slam_toolbox instead of AMCL plus the saved map
- `map` (default `maps/rooms_map.yaml`): full path to a map YAML, ignored when `slam:=true`
- `params_file` (default `config/nav2_params.yaml`): Nav2 parameter file
- `rviz` (default true): open RViz2
- `gui` (default true): show the Gazebo window

Examples:

```bash
ros2 launch nav_sim nav2_sim.launch.py gui:=false
ros2 launch nav_sim nav2_sim.launch.py map:=$HOME/rooms_map.yaml
```

### 6.5 Switch the local planner

Regulated Pure Pursuit (default) follows the global path directly and slows for curves. It needs little tuning.

DWB samples many candidate trajectories and scores them. It reacts more freely to obstacles but needs more tuning and more CPU.

```bash
ros2 launch nav_sim nav2_sim.launch.py \
  params_file:=$HOME/slam_ws/src/nav_sim/config/nav2_params_dwb.yaml
```

### 6.6 What you see in RViz2

- Green line: the global path (`/plan`)
- Red line: the short-term local path (`/local_plan`, DWB only)
- Black and grey: the static map
- Coloured overlays: the global and local costmaps (cyan is inflated cost, purple is inscribed, red is lethal)
- Red dots: lidar hits
- Magenta outline: the robot footprint used for collision checks

### 6.7 Main Nav2 settings

In `nav_sim/config/nav2_params.yaml`:

- Global planner: `NavfnPlanner`, `use_astar: False`, `tolerance: 0.5`
- Controller rate: `controller_frequency: 10.0`
- `failure_tolerance: 1.0` seconds of controller failure allowed before aborting
- Progress checker: the robot must move 0.2 m within 20 s
- Goal tolerances: 0.15 m position, 0.25 rad heading
- Pure Pursuit: `desired_linear_vel: 0.25`, `lookahead_dist: 0.6`
- Costmaps: `robot_radius: 0.25`, `inflation_radius: 0.55`, `cost_scaling_factor: 3.0`
- Speed limits (velocity smoother): 0.25 m/s forward, 0.8 rad/s turning
- AMCL initial pose: (-3.0, 1.5, 0)

The doorways are 1.4 m wide. With the 0.25 m robot radius the robot has about 0.45 m of clear space on each side. If it hesitates at doors, lower `inflation_radius` to 0.45.

---

## 7. The robot

Defined in `slam_sim/urdf/robot.urdf.xacro`.

Body and drive
- Body: box 0.40 x 0.30 x 0.10 m, 5 kg, blue
- Two driven wheels, radius 0.05 m, separation 0.34 m, high friction, with a white stripe so rotation is visible in Gazebo
- Two passive caster spheres (front and rear) with zero friction
- Gazebo diff-drive plugin: converts `/cmd_vel` into wheel velocities and publishes `/odom` plus the `odom -> base_footprint` transform

Lidar
- 360 beams over 360 degrees, 10 Hz
- Range 0.12 to 10 m, Gaussian noise (1 cm standard deviation)
- Publishes `sensor_msgs/LaserScan` on `/scan` in the `lidar_link` frame

Depth camera
- Mounted at the front, 60-degree horizontal field of view
- 160 x 120 pixels at 10 Hz, range 0.3 to 8 m
- Publishes `/camera/image_raw`, `/camera/depth/image_raw` and `/camera/points`
- Uses the optical frame `camera_optical_link` (Z forward, X right, Y down)

Frames
- `base_footprint` is on the ground and is the robot frame used by SLAM and Nav2
- `base_link` is raised 0.075 m so the wheels touch the ground

The depth camera is simulated and can be viewed in RViz2 (enable the DepthCloud and DepthImage displays in the SLAM layout), but it is not used by slam_toolbox or the Nav2 costmaps.

---

## 8. The world

Defined in `slam_sim/worlds/rooms.world`. It uses only simple boxes and cylinders, so no model downloads are needed.

```
 y=+6  +-------------+-------------+
       |   NW room   |   NE room   |
       |             :             |
 y=0   +- - - - - - - + - - - - - -+     : and - - are 1.4 m doorways
       |             :             |     at x = +-3 and y = +-3
       |   SW room   |   SE room   |
 y=-6  +-------------+-------------+
     x=-6           x=0           x=+6
```

- Outer walls enclose 12 x 12 m, 1.5 m high
- Two inner walls cross at the centre, each with two 1.4 m doorways, so the rooms form a loop (useful for loop closure)
- Box and cylinder obstacles (0.8 m high) in every room
- The robot spawns at (-3.0, 1.5) in the NW room, facing east
- Physics step is 4 ms to keep CPU load down

If you change the world, update `nav_sim/tools/generate_map.py` to match and run it to regenerate the navigation map.

---

## 9. Theory in brief

### SLAM

SLAM (Simultaneous Localization and Mapping) lets a robot build a map of an unknown place while estimating its own position in it. At each step the robot predicts its motion from odometry, observes the environment with the lidar, corrects its pose by aligning the new scan with the existing map, and updates the map.

slam_toolbox uses graph-based SLAM. Robot poses at keyframes are nodes in a graph. Edges are constraints from odometry and from scan matching between overlapping scans. When the robot returns to a known place, a loop-closure constraint is added and the Ceres solver adjusts all poses to remove accumulated drift. A new scan is only stored when the robot has moved or turned enough.

The map is an occupancy grid: each 5 cm cell is free (0), occupied (100) or unknown (-1).

### Differential drive

With wheel radius r, wheel separation L and wheel speeds vl and vr:

```
forward speed   v = (vr + vl) / 2
turn rate       w = (vr - vl) / L

odometry:  x += v cos(theta) dt
           y += v sin(theta) dt
           theta += w dt
```

### Navigation

Nav2 splits navigation into cooperating servers:

- AMCL (Adaptive Monte Carlo Localization) tracks the robot's pose on a known map with a particle filter, matching the lidar scan against the map. It publishes the `map -> odom` transform.
- Costmaps turn the map and the live lidar into a grid of traversal costs. Obstacles are inflated by the robot's size so paths keep a safe distance.
- The global planner (NavFn) finds a collision-free path from the robot to the goal over the global costmap.
- The local planner (Pure Pursuit or DWB) follows that path while avoiding obstacles seen in the local costmap, and outputs velocity commands.
- The behaviour tree in `bt_navigator` coordinates planning and control and triggers recovery behaviours (clear costmaps, spin, back up, wait) when the robot gets stuck.

### TF

The transform chain that everything depends on:

```
map -> odom -> base_footprint -> base_link -> lidar_link
                                           -> camera_link -> camera_optical_link
                                           -> left_wheel / right_wheel
```

`map -> odom` comes from slam_toolbox or AMCL (the drift correction). `odom -> base_footprint` comes from the Gazebo diff-drive plugin (smooth but drifting). The rest are fixed transforms from the URDF.

---

## 10. Topics and transform tree

Topics
- `/cmd_vel` (geometry_msgs/Twist): velocity command to the wheels (from teleop or the Nav2 velocity smoother)
- `/odom` (nav_msgs/Odometry): wheel odometry from Gazebo
- `/scan` (sensor_msgs/LaserScan): lidar
- `/camera/image_raw`, `/camera/depth/image_raw` (sensor_msgs/Image): colour and depth images
- `/camera/points` (sensor_msgs/PointCloud2): depth point cloud
- `/map` (nav_msgs/OccupancyGrid): map from slam_toolbox or map_server
- `/goal_pose` (geometry_msgs/PoseStamped): navigation goal from RViz2
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped): AMCL start pose
- `/plan` and `/local_plan` (nav_msgs/Path): global and local paths
- `/global_costmap/costmap` and `/local_costmap/costmap` (nav_msgs/OccupancyGrid): costmaps
- `/tf`, `/tf_static`: transforms

---

## 11. Useful commands

```bash
# Topics and rates
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel

# Transform tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map base_footprint

# Nav2 node states (should all be "active")
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator

# Test the global planner on its own
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
"{goal: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.0}, orientation: {w: 1.0}}}, use_start: false}"

# Save the launch output and look for warnings
ros2 launch nav_sim nav2_sim.launch.py 2>&1 | tee ~/nav.log
grep -E "WARN|ERROR" ~/nav.log | tail -30

# Kill stuck Gazebo processes
killall -9 gzserver gzclient
```

---

## 12. Troubleshooting

General

- Package not found: run `source ~/slam_ws/install/setup.bash` in that terminal.
- Gazebo hangs on first start: it is downloading the sun and ground-plane models. Wait and check the internet connection.
- Gazebo is black or crashes: try `export LIBGL_ALWAYS_SOFTWARE=1` before launching.
- Old Gazebo processes block a restart: `killall -9 gzserver gzclient`.
- Keys respond after a delay, or everything is slow: the Real Time Factor (bottom of the Gazebo window) is low. Use `gui:=false`, keep the DepthCloud display off, close other programs, and give a VM more CPU cores.
- "Message Filter dropping message" or "queue is full": the simulation is too slow, or SLAM or Nav2 started before TF was ready. Wait, reduce the load and confirm `use_sim_time` is true.
- RViz2 says a view class cannot be loaded: the class name is `rviz_default_plugins/TopDownOrtho`, already correct in the provided layouts.

SLAM

- Map is empty in RViz2: check the Map display uses Durability "Transient Local", that `/scan` is publishing and that the robot has moved.
- Map is skewed or doubled: you drove or turned too fast. Slow down and check that the URDF wheel separation and diameter match the diff-drive plugin settings.

Navigation

- Goal is ignored: Nav2 is not active yet or localization is missing. Wait 30 s, check `ros2 lifecycle get /bt_navigator`, and click "2D Pose Estimate" if needed.
- No green path after setting a goal: the global planner failed. Run the `compute_path_to_pose` test above and read the planner error.
- Costmap not shown or incomplete: `map -> odom` is missing. Check `tf2_echo map odom` and set the pose with "2D Pose Estimate".
- Robot model does not line up with the lidar scan: localization is wrong. Re-set the pose with "2D Pose Estimate".
- Robot moves a little, rotates, then stops: the controller failed and recovery ran. Read the `controller_server` warnings in the log ("Failed to make progress", "No valid trajectories", "collision ahead") and try the other local planner.
- Robot refuses to enter a doorway: lower `inflation_radius` to 0.45.
- Robot stops very close to the goal: increase `xy_goal_tolerance` or `yaw_goal_tolerance`.

---

## 13. Known limitations

- slam_toolbox and the Nav2 costmaps use only the 2D lidar. The depth camera is simulated and visualized but not fused into mapping or navigation.
- The lidar and depth camera are simulated on the CPU, so a slow machine or VM can give a low Real Time Factor. This is the most common cause of delayed or erratic behaviour.
- The provided Nav2 parameters are written for ROS 2 Humble, whose `navigation_launch.py` includes a velocity smoother. Other distributions may need parameter changes.
- The AMCL start pose and the automatic initial-pose message match the provided map and spawn position. With your own map, set the pose in RViz2.
- The workspace was written for Gazebo Classic. Gazebo (Ignition / gz sim) is not supported.

---

## 14. Possible extensions

- Convert the depth image to a second laser scan with `depthimage_to_laserscan` and add it to the costmaps
- Use the point cloud as a 3D obstacle source in the costmaps
- Run slam_toolbox in `localization` mode instead of AMCL
- Add waypoint following with Nav2's `FollowWaypoints`
- Add new worlds: copy `rooms.world`, edit it and launch with `world:=/path/to/new.world`
- Replace the CPU lidar with a GPU lidar if your machine has a working GPU
- Add wheel slip or odometry noise to test SLAM robustness
