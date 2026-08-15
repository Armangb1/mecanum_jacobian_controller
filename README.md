# mecanum_jacobian_controller

Chainable `ros2_control` controller plugins implementing **forward and inverse Jacobian kinematics** for a 4-wheel mecanum-drive robot.

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E)
![License](https://img.shields.io/badge/License-Apache_2.0-blue)

Part of the [RoboOmni](https://github.com/Armangb1/roboomni) robot ecosystem.

## Controllers

| Controller | Type | Purpose |
|---|---|---|
| `mecanum_jacobian/JacobianController` | forward kinematics | Reads 4 wheel joint velocities → computes body-frame velocities (x, y, ω) → integrates odometry and publishes `odom → base_link` TF |
| `mecanum_jacobian/InverseJacobianController` | inverse kinematics | Takes body-frame velocity commands (`geometry_msgs/Twist`) → computes 4 wheel velocities → writes them to the wheel command interfaces |

Both are `controller_interface::ChainableControllerInterface` plugins, so they can run standalone (subscribed to `/cmd_vel`) or **chained** — `InverseJacobianController` writing body velocities that `JacobianController` consumes as its reference interfaces.

### Kinematics

For a mecanum robot with wheel half-length `a`, wheel half-width `b`, and wheel radius `r`:

```
Forward Jacobian   J    = [ -1   -1   1   1 ]        Inverse Jacobian   J⁺ = 1/r · [  1  -1  -(a+b) ]
                           [  1   -1  -1   1 ]                                        [  1   1  -(a+b) ]
                           [ 1/(a+b) ...     ]                                        [ -1   1  -(a+b) ]
                                                                                      [ -1  -1  -(a+b) ]

[ω₁ ω₂ ω₃ ω₄]ᵀ = J⁺ · [vx vy ω]ᵀ     (inverse)
[vx vy ω]ᵀ    = r · J · [ω₁ ω₂ ω₃ ω₄]ᵀ   (forward)
```

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `joints` | `string[]` | — | The 4 wheel joints (order: fr, fl, rl, rr) |
| `velocity_command_topic` | `string` | `/cmd_vel` | Twist topic for velocity commands *(inverse only)* |

Robot geometry (`HALF_LENGTH`, `HALF_WIDTH`, `WHEEL_RADIUS`) is compiled in — set it to match your chassis in the headers.

## Integration

Add the controllers to your `controllers.yaml` and spawn them from a launch file:

```yaml
jacobian:
  ros__parameters:
    type: mecanum_jacobian/JacobianController
    joints:
      - chassis_fr_wheel_joint
      - chassis_fl_wheel_joint
      - chassis_rl_wheel_joint
      - chassis_rr_wheel_joint

inverse_jacobian:
  ros__parameters:
    type: mecanum_jacobian/InverseJacobianController
    joints:
      - chassis_fr_wheel_joint
      - chassis_fl_wheel_joint
      - chassis_rl_wheel_joint
      - chassis_rr_wheel_joint
    velocity_command_topic: /cmd_vel
```

```bash
ros2 run controller_manager spawner jacobian
ros2 run controller_manager spawner inverse_jacobian
```

Drive the robot with:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}"
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mecanum_jacobian
source install/setup.bash
```

## Related repositories

- [`roboomni`](https://github.com/Armangb1/roboomni) — workspace, URDF, and system bringup
- [`mecanum_hardware_interface`](https://github.com/Armangb1/mecanum_hardware_interface) — hardware interface the controllers write to

## License

Apache License 2.0. See [LICENSE](LICENSE). This package reuses the chainable-controller structure from [ros2_controllers](https://github.com/ros-controls/ros2_controllers) (PAL Robotics, Apache 2.0); attribution retained in source headers.
