# unitree_a1_joystick
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to unitree_a1_joystick
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch unitree_a1_joystick unitree_a1_joystick.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `~/input/joy` | sensor_msgs::msg::Joy | Raw input from joystick |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `~/output/cmd_vel` | geometry_msgs::msg::TwistStamped | Velocity messege |

### Services and Actions

| Name           | Type                   | Description  |
| -------------- | ---------------------- | ------------ |
| `~/service/gait` | unitree_a1_legged_msgs::srv::Gait  | Client for triggering action like walk, stop, stand... |

### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `update_rate` | int  | Update joy rate |
| `dir_button_deadzone` | double | time to hold button dir button |
| `button_deadzoe`| double | time to hold action button like X, Y, Z |
| `joy_or_dir_button`| int | To control speed you can choose: 0 - linear joy,  1 - dir buttons, -1 : By toppic |
| `velocity_increment`| double | Increment of dir buttons |
| `linear_ratio`| double | linear ratio for x, y joy |
| `angular_ratio` | double | linear ratio for z joy |
| `linear_x_sensitivity` |  double  | sensitivity for linear joy x |
| `linear_y_sensitivity` |  double  | sensitivity for linear joy y |
| `linear_velocity_limit` | double | linear velocity limit |
| `angular_velocity_limit`| double | angular velocity limit |


