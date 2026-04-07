# Drone MPC Control

ROS2 package for 3D Model Predictive Control (MPC) of quadrotor drones using the PX4 flight stack.

## Features

- **3D MPC Controller**: Full 3D position and velocity control using model predictive control
- **PX4 Integration**: Direct communication with PX4 via MicroRTPS bridge
- **Trajectory Generation**: Support for circular, waypoint, and hover trajectories
- **Real-time Optimization**: Gradient-descent based optimization solver for fast control updates
- **Configurable Parameters**: Easily tunable MPC parameters via YAML configuration

## System Requirements

- ROS2 Humble (or later)
- PX4 Autopilot v1.14+
- Eigen3
- C++17 compiler

## Dependencies

```bash
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
sudo apt install -y libeigen3-dev
```

## Installation

### 1. Install PX4 Messages

```bash
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git
```

### 2. Clone This Package

```bash
cd ~/ros2_ws/src
git clone <your-repo-url> drone_mpc_control
```

### 3. Build

```bash
cd ~/ros2_ws
colcon build --packages-select drone_mpc_control px4_msgs
source install/setup.bash
```

## Usage

### Setup PX4

1. Configure PX4 for MicroRTPS:
   ```
   micrortps start -t UDP -i 127.0.0.1 -r 127.0.0.1 -p 8888
   ```

2. Set PX4 to offboard mode (manual via QGroundControl or via MAVLink)

### Launch the MPC Controller

```bash
ros2 launch drone_mpc_control drone_mpc_control.launch.py
```

Or with custom parameters:

```bash
ros2 launch drone_mpc_control drone_mpc_control.launch.py \
    control_rate:=20.0 \
    drone_mass:=1.5 \
    mpc_horizon:=20
```

### Send Trajectory Goals

#### Send a waypoint:

```bash
ros2 topic pub /goal_position geometry_msgs/msg/Point "{x: 5.0, y: 0.0, z: 5.0}"
```

#### Trigger hover:

```bash
ros2 topic pub /hover_command std_msgs/msg/Empty "{}"
```

## Configuration

Edit `config/mpc_params.yaml` to customize controller behavior:

```yaml
mpc_controller:
  ros__parameters:
    control_rate: 20.0          # Control loop frequency (Hz)
    drone_mass: 1.5              # Drone mass (kg)
    mpc_horizon: 20              # Prediction horizon steps
    mpc_dt: 0.05                 # Time step (s)
    
    position_weight: 10.0        # Position error weight
    velocity_weight: 5.0         # Velocity error weight
    acceleration_weight: 1.0     # Acceleration error weight
    control_weight: 0.1          # Control effort weight
    
    min_thrust: 0.0              # Minimum thrust (N)
    max_thrust: 15.0            # Maximum thrust (N)
```

## Architecture

### Nodes

1. **mpc_controller_node**: Core MPC controller computing optimal control inputs
2. **px4_bridge_node**: Manages PX4 offboard mode and sends commands
3. **trajectory_generator_node**: Generates reference trajectories

### Topics

#### Subscribed Topics
- `/fmu/in/vehicle_odometry` (px4_msgs/msg/VehicleOdometry): Drone state from PX4
- `/reference_trajectory` (nav_msgs/msg/Path): Reference trajectory
- `/goal_position` (geometry_msgs/msg/Point): Single waypoint command
- `/hover_command` (std_msgs/msg/Empty): Hover trigger

#### Published Topics
- `/fmu/in/offboard_control_mode` (px4_msgs/msg/OffboardControlMode): Offboard mode
- `/fmu/in/trajectory_setpoint` (px4_msgs/msg/TrajectorySetpoint): Control setpoints

## MPC Controller Details

### Dynamics Model

The controller uses a simplified 3D dynamics model:
- State: [position, velocity, acceleration, angular_velocity] (12 dimensions)
- Control: [thrust_1, thrust_2, thrust_3, thrust_4] (4 rotors)

### Optimization

The MPC solves:
```
min Σ (||pos_error||² + ||vel_error||² + ||acc_error||² + ||control||²)
s.t. thrust_min ≤ thrust_i ≤ thrust_max
```

Using a gradient descent approach for real-time performance.

## Troubleshooting

### Build errors

If Eigen is not found:
```bash
sudo apt install libeigen3-dev
```

### PX4 connection issues

1. Check MicroRTPS is running on PX4
2. Verify UDP ports (default: 8888)
3. Check firewall settings

### Controller not tracking

1. Verify offboard mode is active
2. Check drone mass parameter matches actual drone
3. Adjust MPC weights in config file
4. Ensure trajectory is being published

## License

Apache-2.0

## Contributing

Contributions welcome! Please submit pull requests or open issues.

## References

- [PX4 Autopilot](https://px4.io/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [MicroRTPS Agent](https://docs.px4.io/main/en/computer_vision/rtps.html)
