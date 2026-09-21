# MuJoCo 仿真常见问题（QA）

SO-101 MuJoCo 仿真的原理性问答汇总。按"启动链路 → 模型格式 → 与 ROS 的衔接"组织。

## 一、启动链路

### Q1: MuJoCo Simulate 是怎么拉起来的？

不是独立启动的，是嵌在 `ros2_control_node` 进程里、由硬件插件创建的：

```
ros2 launch so101_moveit_mujoco so101_mujoco.launch.py
  └─ Node(mujoco_ros2_control/ros2_control_node)
       └─ controller_manager 解析 /robot_description 中的 <ros2_control> 标签
            └─ pluginlib 加载 MujocoSystemInterface（libmujoco_ros2_control.so）
                 ├─ on_init()：读 mujoco_model 参数 → mj_loadXML() 加载 MJCF
                 ├─ headless=false → 创建 Simulate App（GLFW 窗口 + 渲染线程）
                 └─ activate() → 物理线程开始 mj_step() 循环
```

日志中可见顺序：`Loading 'mujoco_model' from ...` → `Starting the MuJoCo rendering thread` → `Starting the MuJoCo physics thread`。物理线程与渲染线程解耦，互不占速；控制循环（100 Hz）从物理数据读状态、写指令。MuJoCo 库本身来自 `mujoco_vendor`（`/opt/ros/humble/opt/mujoco_vendor/lib/libmujoco.so`，3.4.0），无需单独安装。

单独查看模型（不经 ROS）：`/opt/ros/humble/opt/mujoco_vendor/bin/simulate <xml路径>`。

### Q2: 为什么 launch 文件里看不到加载 XML 的地方？

刻意分层的结果。launch 只给三样东西：xacro 路径、控制器 YAML、节点本体。MJCF 路径的"源"在 **`urdf/so101_mujoco.ros2_control.xacro`** 的 `<param name="mujoco_model">` 里，launch 里 grep `xml`/`mjcf` 都找不到。链条：launch → xacro 生成 URDF → robot_state_publisher 发布 `/robot_description` → controller_manager 解析 `<ros2_control>` 标签 → 插件读参数 → 加载 MJCF。ros2_control 的设计思想就是硬件配置全部内聚在 URDF 标签里，launch 保持通用。

### Q3: launch 里的 remap `("~/robot_description", "/robot_description")` 是谁用的？

`ros2_control_node` 内部的 controller_manager 自己用。`~` 展开为 `/controller_manager/robot_description`（节点名 + 话题名），Humble 的 controller_manager 通过**订阅话题**（而非读参数）获取 URDF，好处是与 RSP 启动顺序解耦、单一数据源。remap 把订阅重定向到 RSP 发布的 `/robot_description`。日志证据：`Subscribing to '~/robot_description' topic ...` → `Received robot description file`。

### Q4: 为什么 ros2_control_node 需要 robot_description？

它是完全通用的空壳，编译时对具体机器人零认知，全部信息从 URDF 的 `<ros2_control>` 标签解析：① 加载哪个硬件插件；② 插件参数（mujoco_model、headless 等）；③ 关节与指令/状态接口清单（resource manager 据此分配资源）；④ 初值与限位（JTC 用限位做轨迹整形）。没有 URDF 它什么都做不了。

## 二、MJCF 模型格式

### Q5: mjcf/so101.xml 是怎么来的？是转换出来的吗？

都不是，是**从 GitHub 原样下载的**：仓库 [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) 的 `Simulation/SO101/so101_new_calib.xml`，由 LeKiwi 团队用 onshape-to-robot 从 Onshape CAD 导出，与本包 URDF 是同一标定变体。机器上已克隆该仓库时直接复制即可。没有走 URDF→MJCF 自动转换（MuJoCo `compile` 工具或 `robot_description_to_mjcf.sh` 都能转，但 actuator 参数会丢失官方标定的 kp/kv）。包内只有 `so101_scene.xml`（地板+灯光+include，24 行）是手写的。

### Q6: MJCF 的惯性矩阵是必须的吗？

必须有，但不用手写。MJCF 每个可动 body 要求质量>0 且惯性张量正定（满足三角不等式），否则 MuJoCo 报错。两种合法来源：① 显式 `fullinertia`（官方文件已带，从 CAD 生成）；② `compiler inertia="fromgeom"` 由碰撞网格+density 推算（精度差）。本模型用 ①。

### Q7: MJCF 里 body 的 pos/quat 是绝对位置吗？和 URDF 的 rpy 什么关系？

都不是绝对位置，URDF 和 MJCF 全是树状相对变换。URDF 的变换挂在 joint 的 `<origin>` 上（子 link 系在父 link 系下），MJCF 挂在 `<body pos quat>` 上（body 系在父 body 系下），语义等价、只是"边带变换" vs "节点带变换"。本模型两边数字逐元素一致（如 shoulder：URDF `xyz="0.0388353 ..."` = MJCF `pos="0.0388353 ..."`）。注意 **MJCF 四元数顺序是 w x y z**（实部在前），与 ROS 的 x y z w 相反；文件里 e-16 量级的数是浮点残差，无需清理。

### Q8: `<asset>` 段的书写位置有关系吗？

没有。`<asset>` 是纯声明式资源登记表，不参与树结构；写 `<worldbody>` 前后均可（官方文件就写在后面）。asset 本身也没有空间位姿——位姿属于引用它的 `<geom>`（按名字引用，可一处定义多处复用，如 sts3215 网格被 5 个 geom 引用）。唯一要当心的是 `meshdir` 相对主 XML 文件目录解析。

### Q9: 为什么 geom 里 `mesh="sts3215_03a_v1"` 不带 .stl 后缀？

两个属性是两种东西：`<mesh file="...">` 的 file 是**磁盘文件名**（要后缀，格式靠后缀识别）；`<geom mesh="...">` 的 mesh 是**资产引用名**。MJCF 规定 file 类资产（mesh/texture/skin）不写 `name` 时自动以"文件名去路径去扩展名"命名（文档明确机制，已实证），所以 `sts3215_03a_v1.stl` 注册成名为 `sts3215_03a_v1` 的资产。坑：不同目录同名文件会撞名，外部资源建议显式起名。对比 URDF 无资产层，每次写完整路径。

## 三、与 ROS 的衔接

### Q10: urdf/ 目录还有用吗？MuJoCo 不是不读 URDF 吗？

有用，是链路的一半。分工：`urdf/` 给 ROS 侧——robot_state_publisher 算 TF、RViz 渲染（用 `urdf/assets/` 网格）、controller_manager 解析 `<ros2_control>`；`mjcf/` 只给 MuJoCo 物理引擎（连网格都是自己的一份拷贝）。两边通过 **joint 名** 对齐（6 个 actuator 名 = 6 个 ros2_control joint 名）。之前删除的只是内容已并入 xacro 的纯 `so101_new_calib.urdf`，不是整个目录。

### Q11: 环境里的 mujoco_ros_control 包是哪个？MJCF 和 URDF 各自的角色？

实际包名是 `mujoco_ros2_control`（0.0.3，装在 /opt/ros/humble，PAL/NASA/ros-control 维护）。MuJoCo **只读 MJCF**，URDF 只给 controller_manager 提供 `<ros2_control>` 标签，两者靠 joint 名对齐。它提供的 `ros2_control_node` 一个进程内同时跑 MuJoCo Simulate 物理线程和 controller_manager。

## 四、抓取与交互（2026-09-21）

### Q12: 为什么夹爪碰到红色圆柱它纹丝不动？根本抓不起来

MJCF 语义：**body 里没有 `<joint>` 就是焊死在父 body 上**（与 URDF 的 fixed link 不同，MJCF 无关节 body 是静态焊接，不是"质量为零的固定件"——反直觉但文档明确）。场景里的圆柱最初就是这么写的，表现为"接触检测一切正常、力也有、就是不动"。

修复：body 内加 `<freejoint/>`（或 `<joint type="free"/>`）成为自由刚体。改完必须重新 build（MJCF 在 install 里有一份拷贝）+ 重启仿真。注意自由体被臂撞到会滚走，避障场景的位置不再固定。

### Q13: 圆柱"太滑"夹不住，一挤就滚出去

MuJoCo 默认摩擦三元组 `friction="1.0 0.005 0.0001"`（切向/扭转/滚动），**滚动摩擦 0.0001 约等于零**。圆截面自由刚体受单动颚挤压时不是"打滑"而是直接滚出张口。且 `condim` 默认 3（无法向扭转维），想靠扭转摩擦稳住也不行。

修复（`so101_scene.xml` 圆柱 geom）：

```xml
<geom name="cylinder_geom" ... mass="0.05" friction="2.0 0.05 0.05" condim="4"/>
```

两点：① 接触对摩擦取两端较大值，只改圆柱即可；② `condim=4` 必须**显式声明在圆柱端**——手指碰撞 geom 是默认 condim=3，按"取两者较大值"规则不写就不生效。

### Q14: 遥操时夹爪张开角度特别小（只有几度）

量纲错配，不是仿真问题：mujoco 夹爪是 **position 控制器**（`JointGroupPositionController`），指令值是关节位置（rad，限位 [-0.175, 1.745]）；而 xbox_control 默认配置的 `gripper_open_value: 0.1` 是给 gazebo **effort 控制器**（N·m）的。0.1 被当成 0.1 rad ≈ 5.7°，所以"夹角很小"。

修复：遥操时用 mujoco 专用配置（xbox_control 仓库 `joint_teleop/config/mujoco_teleop.yaml`，开 1.5 / 合 -0.1，已实测验证到位）。gazebo 场景继续用默认配置。
