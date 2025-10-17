# depth_port_manager

## Project Description

*Project description*

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `sonia_common_cpp`
* `std_msgs`
* `std_srvs`

## Build Instructions

```bash
colcon build --packages-select depth_port_manager --symlink-install
source install/setup.bash
```
---

## Registered Topics / Services / Actions

| Type                             | Name                      | Direction       | Message/Service Type    | Description                       |
| -------------------------------- | ------------------------- | ----------------| ----------------------- | --------------------------------- |
| Topic                            | `/provider_depth/depth`   | Published       | `std_msgs/msg/Float32`  | Raw depth data                    |
| Topic                            | `/provider_depth/press`   | Published       | `std_msgs/msg/Float32`  | Raw pressure data                 |
| Topic                            | `/provider_depth/temp`    | Published       | `std_msgs/msg/Float32`  | Raw temperature data              |
| Service                          | `/provider_depth/tare`    | Service Server  | `std_srvs/srv/Trigger`  | Depth taring service              |

---

## Launch Instructions

### Default launch

```bash
ros2 launch depth_port_manager launch.py
```

### Alternative launch

```bash
ros2 run depth_port_manager depth_port_manager
```

---

## Useful ROS 2 Commands

```bash
ros2 node list
ros2 node info /depth_port_manager
ros2 topic echo /provider_depth/depth
ros2 param list /depth_port_manager
```

---

## References

* [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
* [sonia_common_ros2](https://github.com/sonia-auv/sonia_common_ros2)

---