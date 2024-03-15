# Copyright 2023 Maciej Krupka
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def action_remap(name: str, remap_str: str):
    return [(f'{name}/_action/{var}', f'{remap_str}/_action/{var}')
            for var in
            ['feedback', 'status', 'cancel_goal', 'get_result', 'send_goal']]


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('unitree_a1_launch')
    unitree_legged_params = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_driver.param.yaml"
    ])
    unitree_legged_launch = ComposableNode(
        package='unitree_a1_driver',
        plugin='unitree_a1_driver::UnitreeLeggedNode',
        name='unitree_a1_driver',
        parameters=[unitree_legged_params],
        remappings=[
                ("/unitree_a1_driver/output/state", "/unitree_a1_legged/state"),
                ("/unitree_a1_driver/output/joy",
                 "/unitree_a1_legged/sensors/joy/data"),
                ("/unitree_a1_driver/input/command", "/unitree_a1_legged/cmd"),
                ("/unitree_a1_driver/output/joint_states",
                 "unitree_a1_legged/joint_states"),
                ("/unitree_a1_driver/output/imu",
                 "unitree_a1_legged/sensors/imu/data_raw"),
                ("/unitree_a1_driver/output/fl_contact",
                 "unitree_a1_legged/sensors/contact_fl"),
                ("/unitree_a1_driver/output/fr_contact",
                 "unitree_a1_legged/sensors/contact_fr"),
                ("/unitree_a1_driver/output/rl_contact",
                 "unitree_a1_legged/sensors/contact_rl"),
                ("/unitree_a1_driver/output/rr_contact",
                 "unitree_a1_legged/sensors/contact_rr"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    unitree_legged_joystick_file = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_joystick.param.yaml"
    ])
    unitree_joystick_launch = ComposableNode(
        package='unitree_a1_joystick',
        plugin='unitree_a1_legged::UnitreeJoystickNode',
        name='unitree_a1_joystick',
        parameters=[unitree_legged_joystick_file],
        remappings=[
                ("/unitree_a1_joystick/output/cmd_vel",
                 "unitree_a1_legged/sensors/joy/cmd_vel"),
                ("/unitree_a1_joystick/input/joy",
                 "/unitree_a1_legged/sensors/joy/data"),
            ("/unitree_a1_joystick/service/gait",
                    "/unitree_a1_legged/service/gait")
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    state_machine = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_highlevel.param.yaml"
    ])
    state_machine_component = ComposableNode(
        package='unitree_a1_highlevel',
        plugin='unitree_a1_highlevel::UnitreeStateMachineNode',
        name='unitree_a1_highlevel',
        parameters=[state_machine],
        remappings=[
            ("/unitree_a1_highlevel/input/twist",
             "/unitree_a1_legged/sensors/joy/cmd_vel"),
            ("/unitree_a1_highlevel/output/twist",
             "unitree_a1_legged/controllers/cmd_vel"),
            ("/unitree_a1_highlevel/service/gait",
             "unitree_a1_legged/service/gait"),
        ] + action_remap('/unitree_a1_highlevel/action/fixed_stand', '/unitree_a1_legged/action/stand') +
        action_remap('/unitree_a1_highlevel/action/hold_position',
                     '/unitree_a1_legged/action/hold_position'),
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    stand_params = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_fixed_stand_server.param.yaml"
    ])
    stand_server_component = ComposableNode(
        package='unitree_a1_fixed_stand_server',
        plugin='unitree_a1_fixed_stand_server::UnitreeFixedStandServerNode',
        name='unitree_a1_fixed_stand_server',
        parameters=[stand_params],
        remappings=[
            ("/unitree_a1_fixed_stand_server/output/cmd",
             "/unitree_a1_legged/cmd"),
            ("/unitree_a1_fixed_stand_server/input/state", "/unitree_a1_legged/state"),
            ("service_name", "/unitree_a1_legged/stand"),
        ] + action_remap('unitree_a1_fixed_stand_server/action/fixed_stand',
                         '/unitree_a1_legged/action/stand') +
        action_remap('unitree_a1_fixed_stand_server/action/hold_position',
                     '/unitree_a1_legged/action/hold_position'),
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    neural_params = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_neural_control.param.yaml"
    ])
    neural_component = ComposableNode(
        package='unitree_a1_neural_control',
        plugin='unitree_a1_neural_control::UnitreeNeuralControlNode',
        name='unitree_a1_neural_control',
        parameters=[neural_params],
        remappings=[
            ("/unitree_a1_neural_control/input/cmd_vel",
             "/unitree_a1_legged/controllers/cmd_vel"),
            ("/unitree_a1_neural_control/input/state", "/unitree_a1_legged/state"),
            ("/unitree_a1_neural_control/output/command",
             "/unitree_a1_legged/cmd"),
            ("/unitree_a1_neural_control/input/imu",
             ("/unitree_a1_legged/sensors/imu/data")),
        ],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    imu_filter_params = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "imu_filter.yaml"
    ])
    imu_madgwick_component = ComposableNode(
        package='imu_filter_madgwick',
        plugin='ImuFilterMadgwickRos',
        name='unitree_a1_imu_filter',
        remappings=[
            ("/imu/data_raw", "/unitree_a1_legged/sensors/imu/data_raw"),
            ("/imu/data", "/unitree_a1_legged/sensors/imu/data"),
        ],
        parameters=[imu_filter_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    container = ComposableNodeContainer(
        name='unitree_a1_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            unitree_legged_launch,
            unitree_joystick_launch,
            state_machine_component,
            stand_server_component,
            neural_component,
            imu_madgwick_component,
        ],
        output='both',
    )
    group = GroupAction([
        container
    ])
    return [group]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('with_rviz', 'False')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
