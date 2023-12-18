# unitree_legged


## Dependecies
```
vcs import < deps.repos
cd /unitree_a1_legged/lcm
mkdir build
cd build
cmake ../
make
```

## Build
```
colcon build --packages-up-to --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On -Wall -Wextra -Wpedantic
```

## Before Launch

Before running the Unitree Legged Node, please follow these steps:

1. **Position the Robot**: Make sure the robot is lying on the ground, and the motors are turned on.

2. **Sit Down the Robot**: Press and hold the L1 button and A button to sit down the robot.

3. **Activate Joint Control Mode**: To control the robot's joints, press both the L1 and L2 buttons simultaneously, and then press the A button.

4. **Ready to Launch**: Once you have completed the above steps, you can proceed to launch the Unitree Legged Node.


```
cd ros2_ws
source /opt/ros/humble/setup.bash
ros2 launch unitree_a1_legged unitree_a1_legged.launch.py
```
**Press Enter to continue...**

## Example
run on second terminal
```
ros2 launch unitree_a1_examples unitree_a1_example_position.launch.py
```

## Worth to know 

If the dynamic library cannot be found, you can use the following command.

```
export LD_LIBRARY_PATH=<PATH::TO::unitree_legged_sdk>/lib:$LD_LIBRARY_PATH
```

[unitree_a1_legged](./unitree_a1_legged/README.md)

[Docker](./docker/README.md)
