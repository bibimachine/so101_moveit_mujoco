"""SO101 MoveIt Servo 启动文件 (MuJoCo 版).

独立于 so101_mujoco.launch.py: 假设 MuJoCo 仿真(控制器/joint_states/TF)已在运行,
本文件只负责把 servo_node 拉起来并调用 start_servo 服务.

前提:
  1. 已启动 MuJoCo 仿真:  ros2 launch so101_moveit_mujoco so101_mujoco.launch.py headless:=true
  2. 已启动手柄遥控链:    ros2 launch joint_teleop joint_teleop.launch.py (xbox_control 工作区)
                          注意: mujoco 夹爪是 position 控制器, 需在自己的 yaml 里把
                          gripper_open/close_value 改成限位内位置值(如 1.5 / -0.1)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_file(path):
    with open(path, 'r') as f:
        return f.read()


def load_yaml(path):
    import yaml
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_share = get_package_share_directory('so101_moveit_mujoco')

    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='servo_node 日志级别, 排查问题时用 debug')
    # 与仿真侧 xacro 展开保持一致 (headless 只影响 ros2_control 硬件参数, 不影响运动学)
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='与 so101_mujoco.launch.py 的 headless 保持一致(robot_description 对齐用)')

    log_level = LaunchConfiguration('log_level')

    # xacro 展开为 robot_description —— mujoco 版顶层 xacro (与 so101.srdf 的关节组对齐;
    # 该 URDF 无 world link, planning_frame 用 base_link, 见 servo_parameters.yaml)
    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'so101.urdf.xacro'),
                 ' headless:=', LaunchConfiguration('headless')]),
        value_type=str)
    srdf = load_file(os.path.join(pkg_share, 'moveit_config', 'so101.srdf'))
    kinematics = load_yaml(os.path.join(pkg_share, 'moveit_config', 'kinematics.yaml'))

    # ServoParameters 的命名空间是 "moveit_servo", 包一层后 flat key
    # 展开为 moveit_servo.publish_period 等 (与官方 demo 相同手法)
    servo_params = {'moveit_servo': load_yaml(os.path.join(pkg_share, 'moveit_config', 'servo_parameters.yaml'))}

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {'robot_description': robot_description},
            {'robot_description_semantic': srdf},
            {'robot_description_kinematics': kinematics},
            servo_params,
            {'use_sim_time': True},
        ],
    )

    # 2.5.9 的 servo_node 启动后处于暂停态, 必须调 start_servo 才开始输出
    start_servo = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/servo_node/start_servo', 'std_srvs/srv/Trigger'],
        output='screen'
    )

    return LaunchDescription([
        log_level_arg,
        headless_arg,
        servo_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=servo_node,
                on_start=[
                    # 等 Servo 内部 PlanningScene/参数初始化完成
                    TimerAction(period=3.0, actions=[start_servo]),
                ])
        ),
    ])
