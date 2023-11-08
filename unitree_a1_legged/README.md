# Unitree A1 legged
The Unitree A1 Legged package provides classes and utilities for interacting with Unitree robots using ROS 2. It facilitates communication with Unitree robots through UDP connections, offering the ability to send commands and receive states.
## Dependecies
- [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk.git) v3.3 
- [unitree_legged_msgs](../unitree_a1_legged_msgs/README.md)

### UnitreeLegged class
The UnitreeLegged class within this package manages the communication with Unitree robots. It allows sending low-level commands and receiving low-level states over a UDP connection.

### UnitreeLeggedNode class

### Output

| Name                            | Type                               | Description       |
| ------------------------------- | ---------------------------------- | ----------------- |
| `unitree_lowlevel\state`        | unitree_legged_msgs::msg::LowState | States of unitree |
| `unitree_lowlevel\joint_states` | sensor_msgs::msg::JointState       | State of joints   |


### Input

| Name      | Type                                | Description |
| --------- | ----------------------------------- | ----------- |
| `command` | unitree_a1_legged_msgs::msg::LowCmd | Commands    |


### Converter
The Converter class provides methods for converting Unitree commands and states to ROS 2 messages and vice versa. It serves as an essential bridge between the Unitree SDK and ROS 2, facilitating seamless communication with Unitree robots.