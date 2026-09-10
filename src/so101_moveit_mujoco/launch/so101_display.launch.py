import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_moveit_mujoco')
    default_model_path = os.path.join(pkg_share, 'urdf', 'so101.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'so101_ros2_control.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'so101_display.rviz')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=str(default_model_path),
        description='SO101 xacro 文件的绝对路径')

    # use_mujoco:=false -> mock_components 硬件，不起 MuJoCo，快速验证控制器配置
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'), ' use_mujoco:=false']),
        value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
            ParameterFile(controllers_yaml),
        ],
        remappings=[('/controller_manager/robot_description', '/robot_description')],
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['so101_joint_state_broadcaster', '--param-file', controllers_yaml],
        output='screen',
    )
    load_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['so101_arm_controller', '--param-file', controllers_yaml],
        output='screen',
    )
    load_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['so101_gripper_controller', '--param-file', controllers_yaml],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        ros2_control_node,
        TimerAction(period=3.0, actions=[load_joint_state_broadcaster]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller])
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_arm_controller,
                on_exit=[load_gripper_controller])
        ),
        rviz_node,
    ])
