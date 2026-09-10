# so101_moveit_mujoco

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.4.0-0066CC)](https://mujoco.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/22.04/)

SO101 机械臂 + ROS 2 + MuJoCo 的学习实践项目。流程对照 [so101_moveit_gazebo](../so101_moveit_gazebo)（Gazebo 版），物理引擎换成 MuJoCo，URDF/xacro、控制器配置、launch 组织方式保持一致。

## 环境

| 项目 | 版本 |
|---|---|
| ROS 2 | Humble |
| MuJoCo | 3.4.0（`mujoco_vendor` 提供） |
| ros2_control 硬件插件 | `mujoco_ros2_control` 0.0.3 |
| 系统 | Ubuntu 22.04（WSL2/WSLg） |
| Python | 3.10 |

## 目前做了什么

- [x] SO101 完整 URDF/xacro 描述（5 臂 + 单活动指夹爪，STL 网格，mesh 用 `package://`）
- [x] MJCF 模型接入（GitHub 官方 SO-ARM100 仓库下载，含标定舵机参数）
- [x] ros2_control 双硬件后端：mock（快速显示）和 MuJoCo（物理仿真）
- [x] MuJoCo Simulate 窗口 + headless 模式 + RViz 显示
- [x] 数据采集：`ros2 bag record` 封装脚本（mcap，话题含控制指令，对齐 LeRobot 数据集要素）
- [x] 单目相机：MJCF 固定相机 + ros2_control `<sensor>` 声明，发布 RGB/深度/camera_info（与 gazebo 版同位姿同话题）

## TODO

- [ ] 接入 MoveIt 2（运动规划、碰撞检测）

## 快速开始

```bash
cd so101_moveit_mujoco
colcon build && source install/setup.bash

# MuJoCo 仿真（弹 MuJoCo Simulate 窗口 + RViz）
ros2 launch so101_moveit_mujoco so101_mujoco.launch.py

# 无 GUI 环境（SSH/容器）
ros2 launch so101_moveit_mujoco so101_mujoco.launch.py headless:=true

# 不启动 RViz
ros2 launch so101_moveit_mujoco so101_mujoco.launch.py rviz:=false

# 纯显示（mock 硬件，启动快，不起 MuJoCo）
ros2 launch so101_moveit_mujoco so101_display.launch.py
```

## 控制示例

```bash
# 手臂：JointTrajectoryController（5 关节，2 秒到达）
ros2 topic pub --once /so101_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll], \
    points: [{positions: [0.5, -0.8, 0.9, -0.4, 1.0], time_from_start: {sec: 2}}]}"

# 夹爪：JointGroupPositionController
ros2 topic pub --once /so101_gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.8]}"

# 手动控制台（菜单式关节微调 + 夹爪开合；世界里固定有一个红色圆柱障碍物，
# 写在 MJCF so101_scene.xml 里，与 gazebo 版 spawn 的位置相同）
ros2 run so101_moveit_mujoco manual_control.py
```

## 相机图像

MJCF `so101_scene.xml` 里声明了固定单目相机（与 gazebo 版同位姿、同视角），
由 mujoco_ros2_control 离屏渲染并发布（10 Hz）：

- `/so101_camera/image_raw`（RGB，`sensor_msgs/Image`）
- `/so101_camera/depth/image_raw`（深度图）
- `/so101_camera/camera_info`（内参，`frame_id` 为 `so101_camera_optical_frame`，TF 已发布）

```bash
# cv2 窗口显示（标题栏显示 FPS，q 退出）
ros2 run so101_moveit_mujoco show_fps.py
# 或用 image_view
ros2 run image_view image_view_node image:=/so101_camera/image_raw
```

## 数据采集

需先启动 `so101_mujoco.launch.py`（默认话题含 `/joint_states`、TF、控制指令，相机话题见上一节，
录 bag 时可用 `-t "/joint_states /tf /tf_static /clock /so101_camera/image_raw /so101_camera/camera_info"` 加上图像）：

```bash
ros2 run so101_moveit_mujoco record_so101.sh -h        # 查看选项
ros2 run so101_moveit_mujoco record_so101.sh -d 120    # 每 120s 切分一个 bag
```

## 文档

- [doc/文件说明.md](doc/文件说明.md) — 各文件作用、控制命令、与 gazebo 版架构对照
- [doc/mujoco_qa.md](doc/mujoco_qa.md) — MuJoCo 原理问答（启动链路、MJCF 格式机制等）
- [doc/control_basics.md](doc/control_basics.md) — 控制基础（PID 原理、弧度/度、Gazebo 与 MuJoCo 差异）
