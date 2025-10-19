# depth_port_manager

The project enables a serial connection with the depth sensor **Impact Subsea ISD4000** for data transmission. The connection reads raw measurement data which is then parsed into depth, pressure, and temperature and then transmitted using **ROS2** communication to other systems on the network.

---

## Dependencies

### ROS 2 Distro

* Humble

### ROS 2 Packages

* `ament_cmake`
* `rclcpp`
* `std_msgs`
* `std_srvs`

### Additional packages

* `Boost`
* `sonia_common_cpp`

---

## Node

* Name: `depth_provider`
* Port Name: `/dev/DEPTH`
* Port type: serial
* Baud Rate: 115200

---

## Registered Topics / Services / Actions

| Type                             | Name                      | Direction       | Message/Service Type    | Description                                 |
| -------------------------------- | ------------------------- | ----------------| ----------------------- | ------------------------------------------- |
| Topic                            | `/provider_depth/depth`   | Published       | `std_msgs/msg/Float32`  | Measured depth data                     |
| Topic                            | `/provider_depth/press`   | Published       | `std_msgs/msg/Float32`  | Measured pressure data                  |
| Topic                            | `/provider_depth/temp`    | Published       | `std_msgs/msg/Float32`  | MEasured temperature data               | 
| Service                          | `/provider_depth/tare`    | Service Server  | `std_srvs/srv/Trigger`  | Resets the depth sensor to current position |

---
## Build Instructions
To build the project, the following commands should be run directly from your ROS2 workspace.

```bash
colcon build --packages-select depth_port_manager --symlink-install
source install/setup.bash
```
---

## Launch Instructions

### Default launch

```bash
ros2 launch depth_port_manager launch.py
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
* [ImpactSubsea ISD4000 manual](https://www.impactsubsea.co.uk/wp-content/uploads/2020/01/ISD4000-Manual-Rev-1.7.pdf)

---