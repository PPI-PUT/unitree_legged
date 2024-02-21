# unitree_a1_legged_launch
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --packages-up-to unitree_a1_legged_launch
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch unitree_a1_legged_launch unitree_a1_legged_launch.launch.py
```

## Description
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Config 
Config directory contains configs file. To change the parameters, make them here.

### Launch

- unitree_launch.launch.py : run whole stack
- rviz_launch.launch.py : run visualization of states in rviz
- rviz_nn_launch.launch.py : vizualization of control commands from nn controller
- gazebo_launch.launch.py : run simulator in gazebo

