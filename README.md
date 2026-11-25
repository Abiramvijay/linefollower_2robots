# Multi-Robot Colour Line Follower (ROS 2 & Gazebo Fortress)

A ROS 2 Humble + Gazebo Fortress simulation where two differential-drive robots (`robot1` and `robot2`) autonomously track separate **red** and **blue** paths using downward-facing RGB cameras, OpenCV HSV segmentation, and PID steering control, with a Tkinter control panel for starting, stopping, and returning both robots along their recorded paths.

## Features

- **Dual-Robot Simulation:** Spawns two independent differential-drive robots (`robot1` on the red track, `robot2` on the blue track) inside `multi_robot_world`.
- **Vision-Based Line Tracking:** Uses OpenCV HSV colour thresholding, morphological filtering, and bottom-60% ROI contour/centroid detection to compute normalised lateral tracking error.
- **PID Steering & Adaptive Speed:** Closed-loop `PIDController` for angular velocity (`cmd_vel.angular.z`) with adaptive linear velocity reduction on sharp turns.
- **Line-Loss Recovery & Safe Stop:** Briefly searches in the last known error direction when the line disappears and halts safely (`lost_line`) after `line_missing_timeout` seconds.
- **Odometry Path Recording & Return Home:** Records waypoints every `0.06 m` during forward tracking and reverses through the recorded waypoints back to the start position when `/return_home` is triggered.
- **GUI Race Controls:** A Tkinter window (`race_controls`) provides one-click **START BOTH ROBOTS**, **STOP BOTH ROBOTS**, and **RETURN BOTH ROBOTS HOME** controls.

---

## Workspace Structure

```text
multi_robot_ws/
├── README.md
└── src/
    ├── multi_robot_description/          # Gazebo world, robot SDF model, and simulation bringup
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── launch/
    │   │   ├── bringup.launch.py         # Launches Gazebo world, bridge, controllers, and GUI
    │   │   └── multi_robot_world.launch.py
    │   ├── models/
    │   │   └── line_follower_robot/
    │   │       ├── model.config
    │   │       └── model.sdf             # DiffDrive robot + 640x480 downward camera
    │   └── worlds/
    │       └── multi_robot_world.sdf     # 12.4m x 8.4m arena with red_path and blue_path
    └── multi_robot_controller/           # ROS 2 Python controllers and GUI
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── config/
        │   └── line_follower_params.yaml # Reference parameter configuration
        ├── launch/
        │   └── line_follower.launch.py   # Launches robot1, robot2 controllers & race_controls
        ├── multi_robot_controller/
        │   ├── __init__.py
        │   ├── line_follower_node.py     # Main vision + PID + odometry return node
        │   ├── pid_controller.py         # PID controller & angle/clamp utilities
        │   └── race_controls.py          # Tkinter GUI publishing /start_race & /return_home
        ├── resource/
        │   └── multi_robot_controller
        └── test/
            ├── test_copyright.py
            ├── test_flake8.py
            └── test_pep257.py
```

---

## Prerequisites

- **OS / ROS:** Ubuntu 22.04 with **ROS 2 Humble**
- **Simulator:** Gazebo Fortress with `ros_gz_sim` and `ros_gz_bridge`
- **ROS / Python Dependencies:**
  - `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_msgs`, `cv_bridge`
  - `python3-opencv` (`opencv-python`), `numpy`, `python3-tk`

---

## Build

```bash
source /opt/ros/humble/setup.bash
cd multi_robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Launch Full Simulation & Controllers

```bash
ros2 launch multi_robot_description bringup.launch.py
```

#### Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `headless` | `false` | Run Gazebo without the GUI (`-s -r`) when set to `true`. |
| `show_controls` | `true` | Launch the Tkinter `race_controls` window. |

Example (headless simulation without GUI controls):

```bash
ros2 launch multi_robot_description bringup.launch.py headless:=true show_controls:=false
```

### Launch World and Controllers Separately

```bash
# Terminal 1: Gazebo world + ros_gz_bridge + robot spawns
ros2 launch multi_robot_description multi_robot_world.launch.py

# Terminal 2: Line-follower nodes + race_controls GUI
ros2 launch multi_robot_controller line_follower.launch.py
```

### Controlling Robots via CLI (Without GUI)

```bash
# Start both robots
ros2 topic pub --once /start_race std_msgs/msg/Bool "{data: true}"

# Stop both robots
ros2 topic pub --once /start_race std_msgs/msg/Bool "{data: false}"

# Command both robots to return along their recorded paths
ros2 topic pub --once /return_home std_msgs/msg/Bool "{data: true}"
```

---

## Robot & World Details

- **Spawn Poses (`multi_robot_world.launch.py`):**
  - `robot1` (red line): `(-5.3, -3.3, 0.0)`, yaw `1.47 rad`
  - `robot2` (blue line): `(-5.3, -2.55, 0.0)`, yaw `1.47 rad`
  - Robots are spawned `5.0 s` after Gazebo starts.
- **Robot Model (`models/line_follower_robot/model.sdf`):**
  - Chassis: `0.30 m × 0.24 m × 0.08 m`, `2.0 kg`
  - Wheels: `0.04 m` radius, `0.32 m` separation, `gz::sim::systems::DiffDrive` (`odom_publish_frequency: 30 Hz`)
  - Camera: `640×480` RGB (`R8G8B8`) at `30 Hz`, `1.2 rad` horizontal FOV, mounted at `(0.18, 0, 0.02)` pitched downward (`1.5708 rad`)

---

## ROS 2 Interfaces

### Bridged & Controller Topics

| Topic | Message Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/robot1/camera/image`, `/robot2/camera/image` | `sensor_msgs/msg/Image` | Gazebo → Controller | Downward-facing RGB camera stream |
| `/robot1/odom`, `/robot2/odom` | `nav_msgs/msg/Odometry` | Gazebo → Controller | Robot odometry for distance & path recording |
| `/robot1/cmd_vel`, `/robot2/cmd_vel` | `geometry_msgs/msg/Twist` | Controller → Gazebo | Linear and angular velocity commands |
| `/robot1/status`, `/robot2/status` | `std_msgs/msg/String` | Controller → User | State updates formatted as `<robot_name>:<status>` |
| `/start_race` | `std_msgs/msg/Bool` | Controls → Controllers | `true` starts tracking; `false` stops and resets state |
| `/return_home` | `std_msgs/msg/Bool` | Controls → Controllers | `true` triggers reverse waypoint navigation home |

### Status Values

Each robot publishes `<robot_name>:<status>` on `/<robot_name>/status`:
- `waiting_for_start` — Idle and waiting for `/start_race` (`true`)
- `tracking` — Actively following its assigned line colour
- `completed` — Travelled distance reached `path_length`
- `lost_line` — Line remained undetected beyond `line_missing_timeout`
- `returning_home` — Navigating backwards through recorded odometry waypoints
- `home` — Reached the start of the recorded path
- `home_unavailable` — `/return_home` requested before at least 2 waypoints were recorded

### Launch Parameters (`line_follower.launch.py`)

| Parameter | Default (Launch) | Description |
| :--- | :--- | :--- |
| `robot_name` | `robot1` / `robot2` | Robot identifier used for logging and status strings |
| `line_color` | `red` / `blue` | HSV colour target (`red` or `blue`) |
| `base_linear_velocity` | `0.18` | Nominal forward speed (`m/s`) |
| `max_linear_speed` | `0.22` | Maximum linear velocity (`m/s`) |
| `max_angular_velocity` | `1.5` | Maximum angular velocity (`rad/s`) |
| `kp` / `ki` / `kd` | `1.0` / `0.01` / `0.08` | PID steering gains |
| `controller_dt` | `0.05` | Control time step (`s`) used by PID and timeout |
| `line_missing_timeout` | `1.2` | Seconds before stopping when the line is lost |
| `path_length` | `24.9` | Odometry distance (`m`) at which a run completes |
| `track_min_area` | `80` | Minimum contour area (pixels) to accept a line detection |
| `return_speed` | `0.14` | Linear speed (`m/s`) while returning home (node default) |

---

## Testing

Run the `ament` lint tests for `multi_robot_controller`:

```bash
colcon test --packages-select multi_robot_controller
colcon test-result --verbose
```

---

## License

Apache-2.0
