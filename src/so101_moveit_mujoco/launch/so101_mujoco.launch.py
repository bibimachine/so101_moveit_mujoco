import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
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
    headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='true 时不弹出 MuJoCo Simulate 窗口（无 GUI 环境必选）')
    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='是否启动 RViz')

    # 运行 xacro 生成 robot_description（headless 传给 ros2_control 硬件标签）
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model'),
                 ' headless:=', LaunchConfiguration('headless')]),
        value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }]
    )

    # mujoco_ros2_control 的 ros2_control_node：内部同时运行 MuJoCo 物理线程
    # 和 controller_manager。Humble 下必须把 ~/robot_description remap 到 /robot_description
    ros2_control_node = Node(
        package='mujoco_ros2_control',
        executable='ros2_control_node',
        emulate_tty=True,
        output='screen',
        parameters=[
            {'use_sim_time': True},
            ParameterFile(controllers_yaml),
        ],
        # 直接没让gazebo插件转发
        remappings=[('~/robot_description', '/robot_description')],
    )

    # 控制器链式加载：ros2_control_node 常驻，先等它起来（3s），
    # 之后用 spawner 退出事件串联下一个，顺序与 gazebo 版一致
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
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        model_arg,
        headless_arg,
        rviz_arg,
        robot_state_publisher_node,
        ros2_control_node,
        rviz_node,
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
    ])
