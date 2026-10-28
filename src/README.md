# SLAM Simulation — ROS 2 Humble & Gazebo Classic

A differential-drive mobile robot simulation using ROS 2 Humble, Gazebo Classic, and SLAM Toolbox. The project features a custom 12 × 12 m indoor world with four rooms and obstacles, a 2D LiDAR for mapping, and a depth camera for visual perception.

## Features

- Differential-drive mobile robot modeled using URDF/Xacro
- Gazebo Classic indoor simulation
- Custom 12 × 12 m world with four rooms and obstacles
- 2D LiDAR integration for real-time SLAM
- Depth camera for image, depth, and point cloud data
- SLAM Toolbox for 2D occupancy grid mapping
- RViz2 visualization
- Keyboard-based teleoperation
- Occupancy grid map export for future navigation tasks

## Technology Stack

| Component | Technology |
|---|---|
| Operating System | Ubuntu 22.04 |
| Middleware | ROS 2 Humble |
| Simulator | Gazebo Classic |
| Robot Description | URDF / Xacro |
| SLAM | SLAM Toolbox |
| Visualization | RViz2 |
| Mapping Output | ROS 2 OccupancyGrid |

## Project Structure

```text
slam_ws/
├── README.md
└── src/
    └── slam_sim/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/
        │   └── slam_toolbox.yaml
        ├── launch/
        │   ├── bringup.launch.py
        │   ├── sim.launch.py
        │   └── slam.launch.py
        ├── rviz/
        │   └── slam.rviz
        ├── urdf/
        │   └── robot.urdf.xacro
        └── worlds/
            └── rooms.world
```

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Classic 11

Install the required dependencies:

```bash
sudo apt update

sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-slam-toolbox \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-nav2-map-server \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher
```

## Installation and Build

### 1. Navigate to the workspace

```bash
cd ~/slam_ws/slam_ws
```

### 2. Build the package

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 3. Source the workspace

```bash
source install/setup.bash
```

## Launch the Simulation

Start the complete simulation using the main bringup launch file:

```bash
ros2 launch slam_sim bringup.launch.py
```

The bringup launch file is intended to coordinate the simulation and SLAM components. The exact nodes started depend on its implementation.

### Individual Launch Files

The package contains three launch files for modular execution.

**Gazebo simulation:**

```bash
ros2 launch slam_sim sim.launch.py
```

**SLAM Toolbox:**

```bash
ros2 launch slam_sim slam.launch.py
```

**Complete bringup:**

```bash
ros2 launch slam_sim bringup.launch.py
```

Use the individual launch files when you want to debug or run simulation and mapping separately.

## Robot Teleoperation

Open a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/slam_ws/slam_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use the keyboard instructions displayed in the terminal to control the robot.

Explore the environment gradually, passing through doorways and visiting all four rooms. Avoid excessive rotation or rapid movement to improve mapping quality.

## SLAM Mapping

SLAM Toolbox processes LiDAR scan data together with the robot's odometry and TF transforms to estimate the robot's pose and build a 2D occupancy grid.

The primary mapping topic is:

```text
/map
```

The SLAM parameters are stored in:

```text
src/slam_sim/config/slam_toolbox.yaml
```

The RViz2 configuration is stored in:

```text
src/slam_sim/rviz/slam.rviz
```

## Visualize in RViz2

The project includes a predefined RViz2 configuration. When the launch file loads it, inspect the following displays:

- **RobotModel:** Robot geometry
- **TF:** Coordinate frames
- **LaserScan:** LiDAR measurements
- **Map:** Generated occupancy grid
- **Image / DepthImage:** Camera image and depth data
- **PointCloud2:** 3D point cloud, if published

If necessary, add the camera displays manually and select the corresponding topics. Available display types depend on the installed RViz2 plugins.

## Important ROS 2 Topics

| Topic | Purpose |
|---|---|
| `/cmd_vel` | Velocity commands |
| `/odom` | Robot odometry |
| `/scan` | 2D LiDAR data |
| `/camera/image_raw` | Camera image |
| `/camera/depth/image_raw` | Depth image |
| `/camera/points` | Point cloud |
| `/map` | SLAM occupancy grid |

These are the expected topic names. Actual names depend on the sensor plugins, namespace settings, and topic remappings.

Inspect active topics:

```bash
ros2 topic list
```

Check LiDAR data:

```bash
ros2 topic echo /scan
```

Check the map:

```bash
ros2 topic echo /map
```

## Save the Map

After exploring the environment and generating a satisfactory map, open a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/slam_ws/slam_ws/install/setup.bash
```

Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/rooms_map
```

Expected output files:

```text
rooms_map.yaml
rooms_map.pgm
```

The YAML file contains map metadata, while the PGM file contains the occupancy grid image.

## Troubleshooting

### Gazebo does not start

Verify the Gazebo ROS integration packages are installed and inspect the terminal for plugin or world-loading errors.

### The robot does not move

Check that `/cmd_vel` receives commands and that the Gazebo differential-drive plugin subscribes to the correct velocity topic.

```bash
ros2 topic info /cmd_vel
```

### LiDAR is not visible

Verify the scan topic and TF frames:

```bash
ros2 topic list
ros2 topic echo /scan
ros2 run tf2_tools view_frames
```

### SLAM does not generate a map

Check that:

- LiDAR scans are publishing correctly.
- Odometry is available.
- The required `odom`, `base_link`, and LiDAR frame transforms exist.
- The frame names and scan topic in `slam_toolbox.yaml` match the robot configuration.

### Map saving fails

Ensure `/map` is publishing a valid occupancy grid and the destination directory is writable.

## Future Enhancements

- Nav2 autonomous navigation
- Localization using a previously saved map
- Frontier-based autonomous exploration
- Improved odometry and sensor fusion
- Depth-camera-based obstacle perception
- Dynamic obstacles and more realistic sensor simulation

## Learning Outcomes

This project demonstrates practical skills in ROS 2, robot modeling, Gazebo simulation, differential-drive control, LiDAR integration, TF frame management, SLAM Toolbox, RViz2 visualization, and occupancy grid mapping.

## License

Add an appropriate license before distributing the project. Ensure that any third-party assets comply with their respective licenses.

