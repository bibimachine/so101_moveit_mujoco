# MoveIt Servo 接入记录(SO101 + MuJoCo)

> 日期:2026-09-19。环境:ROS 2 Humble,moveit2 源码构建于 `~/ws_moveit2`(humble 分支,moveit_servo 2.5.9)。
> 配套文件:`moveit_config/servo_parameters.yaml`、`launch/so101_moveit_servo.launch.py`。
> **本文只记录与 gazebo 版的差异**,通用流程(启动顺序、全链路对账、手柄映射、版本备忘、时间戳教训)
> 见 `so101_moveit_gazebo` 仓库的 `doc/moveit_servo_接入记录.md`,不重复。

## 1. 与 gazebo 版的差异汇总

| 项目 | gazebo 版 | mujoco 版 | 原因 |
|---|---|---|---|
| `use_gazebo` | `true`(直读 gazebo 关节状态) | 不设(默认 `false`) | mujoco 侧状态来自 `/joint_states`(`so101_joint_state_broadcaster` 发布,50Hz) |
| `planning_frame` | `world` | `base_link` | mujoco URDF(`so101.urdf.xacro`)**无 world link**,root 即 `base_link`,TF 树里没有 world 帧 |
| SRDF virtual joint | `world_joint`(fixed, world→base_link) | 无 | 同上,SRDF 里去掉 virtual_joint 与 group 中的 world_joint |
| URDF 关节 damping | 5 臂关节加 `damping="1.0"` | **不加** | gazebo 位置控制是 10ms 硬传送,窗口内重力下拉产生锯齿抖动;MuJoCo 是连续物理 + position actuator 维持,无此问题 |
| 关节限位 | URDF 单独描述文件 | 与运动学同文件(`so101.urdf.xacro`) | 同一套标定参数,数值完全一致,直接沿用 gazebo 版 joint_limits.yaml |
| 输出控制器 | `so101_arm_controller`(JTC) | `so101_arm_controller`(JTC),**同名** | 两工作区控制器命名对齐,`command_out_topic` 不用改 |
| 夹爪控制器 | effort(`JointGroupEffortController`) | **position**(`JointGroupPositionController`) | ⚠️ 见下面"夹爪指令值" |
| 奇异阈值 | `lower=100 / hard=800` | 同左(沿用标定) | 同为 5DOF 臂,6×5 雅可比条件数特性相同 |
| kinematics.yaml | `{}`(禁用 KDL,回退逆雅可比) | 同左 | 同因:KDL 近似解吃掉平移指令(详见 gazebo 版记录第 6 节) |
| `publish_period` | `0.01`(100Hz) | **`0.05`(20Hz)** | ⚠️ 见下面第 5 节:jog 不动的根因与 40% 速度天花板 |

## 2. 启动顺序

```bash
# 终端1: MuJoCo 仿真(headless 无 GUI; 控制器 / joint_states / TF / /clock)
ros2 launch so101_moveit_mujoco so101_mujoco.launch.py headless:=true rviz:=false

# 终端2: 手柄遥控链(xbox_control 工作区,推荐关节级 C++ 版;
# 注意用 mujoco 专用配置, 默认配置的夹爪值是 gazebo effort 量纲)
ros2 launch joint_teleop joint_teleop.launch.py \
    config_file:=$(ros2 pkg prefix joint_teleop)/share/joint_teleop/config/mujoco_teleop.yaml

# 终端3: Servo(自动调用 /servo_node/start_servo)
ros2 launch so101_moveit_mujoco so101_moveit_servo.launch.py headless:=true
```

启动后可观察:`ros2 topic echo /servo_node/status`(0=静止,1=伺服中,3=碰撞减速,4=碰撞急停)。

## 3. 夹爪指令值(mujoco 专属坑)

gazebo 版夹爪是 effort 控制器,`joint_teleop`/`ee_teleop` 默认发 `0.1 / -0.1`(N·m)。
mujoco 版夹爪是 **position 控制器**(`so101_gripper_controller`),要的是**关节限位内的位置值**
(URDF 限位 `lower=-0.174533, upper=1.74533`)。

用 xbox_control 遥操时**必须**用 mujoco 专用配置(xbox_control 仓库
`joint_teleop/config/mujoco_teleop.yaml`, 已内置开 1.5 / 合 -0.1):

```bash
ros2 launch joint_teleop joint_teleop.launch.py \
    config_file:=$(ros2 pkg prefix joint_teleop)/share/joint_teleop/config/mujoco_teleop.yaml
```

直接发 effort 值 0.1/-0.1 在 position 控制器下是"把关节目标设到限位附近的微小值",
表现为**夹角很小**(开只有 0.1 rad ≈ 5.7°)。仿真内已实测验证:
指令 1.5 rad → 关节到 1.5 rad(85.9°);指令 -0.1 → 到位。

## 4. 碰撞检查的适用范围

Servo 的 PlanningScene 初始为空。MJCF 场景(`so101_scene.xml`)里的**圆柱只存在于 MuJoCo 物理中,
不在 Servo 的 PlanningScene 里**,不会触发场景碰撞减速/急停;**自碰撞检查照常生效**(按 URDF 碰撞网格)。
要场景避障需另行向 `/planning_scene` 发布障碍物(与 gazebo 版一致)。

## 5. jog 速度问题排查(2026-09-20,重要)

### 5.1 现象

原配置(`publish_period: 0.01`,与 ros2_control update_rate=100Hz 对齐)下,JointJog 摇杆
关节速度指令 0.5 rad/s,实测只有 **0.03 rad/s(6%)**,基本爬行;且未指令关节有微小漂移。

### 5.2 根因一(主因):JTC "替换语义 + 首采样锚定" 与相位锁定

ros2_controllers 2.53.1 JTC 的 topic 路径(`~/joint_trajectory`,servo 的默认输出路径)语义:

1. **新消息整体替换旧轨迹,无队列缓冲**;
2. **首采样时才把段起点锚定到当前状态**(stamp=0 单点段 time_from_start=0.01)。

100Hz 短段流下:每条消息 10ms 就被下一条顶掉,一生只被采样一次,且恰好落在
"首采样锚定、进度 ≈ 0%" 的时刻——执行的位置指令 ≈ 原地不动。
数学上每条段的平均有效进度 ≈ 0,故速度趋近 0。

**gazebo 版为什么同配置能用**:gazebo 的 CM 用 wall-clock、servo 用 sim-time,两者相位
缓慢漂移,消息偶尔"活到"被采样到段结尾(进度 →100%),平均下来速度正常。
mujoco 的 CM 与 servo 同源 `/clock`,**相位锁死**,每条消息都精确死在同一个相位点,
问题被放大到全量。

### 5.3 修复与速度天花板

把 `publish_period` 加大,让每条段寿命跨过多个 JTC 消费周期:

| publish_period | 段寿命 / JTC 周期 | 实测(pan,指令 0.5 rad/s) | 达成率 | 未指令关节 |
|---|---|---|---|---|
| 0.01s(原配置) | 1 | 0.03 rad/s | 6% | 漂移 |
| **0.05s(现配置)** | 5 | 0.197 rad/s | **39%** | 干净 |
| 0.1s | 10 | 0.218 rad/s | 44% | 干净 |

0.05s 的 39% 是**该输出路径的理论天花板**:段寿命 50ms = 5 个采样,首采样进度 0,
其余进度 20/40/60/80%,均值 40%。再降频率(0.1s→44%)收益边际,延迟却到 100ms,
摇杆手感变钝,故取 0.05s。10Hz/两点自包含轨迹形态实测可达 78%,但 servo 2.5.9
`composeJointTrajMessage` 硬编码单点 stamp=0,发不了该形态。

**补偿方案(未实施)**:链路实际速度 ≈ 指令 × 0.4 且线性度良好(指令 1.1 rad/s 实测
0.46 rad/s)。要还原 gazebo 手感,把 joint_teleop 的 `omega_max` 从 1.5 提到 ~3.0
(xbox_control 仓库 `config/joint_teleop.yaml`)。代价:碰撞/限位减速区手感同比放大,
且速度更快更容易怼进 5.4 的贴地 stuck 区。

### 5.4 根因二(排查中发现的仿真侧问题,非控制 bug):夹爪贴障碍物/地板 stuck

排查中出现大量"灵异"现象,最终用**纯 MuJoCo 复现**(不经 ROS,直接 `d.ctrl` 给位置)
证明全部是**物理接触**,与 servo/JTC 无关:

- pan 摇到 -0.19 附近"神秘卡死"、目标前进 1/15、全关节冻结
  → 夹爪正好**站在场景的红圆柱上**(0.5mm 压入产生 13N 接触力,举臂力矩被抵消);
- lift 指令 0.9 永远停在 0.38~0.48,执行器 effort **持续饱和 3.35 N·m**(forcerange 上限)
  → 夹爪手指**贴着地板**形成 stuck 平衡。MJCF 执行器 kp=998,接触极硬(0.5mm→13N),
  一旦贴住很难自解;
- **FollowJointTrajectory action 返回 error_code=0 不代表到位**——JTC 只回报"轨迹执行完",
  不检查跟踪误差,物理被挡也报 SUCCESS。观测只信 `/joint_states` 的 position;
- 未指令关节的"漂移"也是接触力在多关节间的再分配(实测漂移比值与接触力臂比值一致)。

**实用结论**:
- lift≈0.4~0.5、flex≈0.25~0.3 这一族姿态,腕部低、手指接近地面,**是 stuck 高发区**;
  摇杆遥操拖行时极易滑进去(本仓库所有 pan 测试都是拖着地板跑的)。
- 去安全位姿要走"**先折小臂(flex↑)抬高手腕,再抬大臂(lift)**"的两步路径;
  单点直插目标若路径扫过贴地区,会把臂卡进 stuck。
- 圆柱障碍物只在 MuJoCo 物理里(见第 4 节),Servo/PlanningScene 不知道它;
  臂被它挡住时 servo 状态仍是 NO_WARNING——**状态话题不能替代对夹爪位置的直接观察**
  (`tf2_echo base_link gripper_frame_link`)。

### 5.5 备用方案评估:去掉 servo 直控 ros2_control(未采用)

备用思路是绕开 servo,把摇杆积分成位置直发 ros2_control。评估结论:

| 能力 | 直控后的情况 |
|---|---|
| 碰撞检测(自碰撞/环境) | **丢失**。这是 servo 每 10ms 查 PlanningScene 的软件层功能,JTC/物理层都没有 |
| 奇异点检测/减速 | **丢失**(同上,servo 按雅可比条件数限速/急停) |
| 关节限位 | 仿真里还在(MuJoCo `limited=true` 物理硬限位);但 JTC 不查 URDF 限位,**实机上必须自实现发送前 clamp**,否则可能打坏电机 |
| JTC 替换语义问题 | **依然存在**。topic 流式发轨迹还是每条被顶掉;action 流式发会打爆 preempt;唯一绕法是 Float64MultiArray + JointGroupPositionController(丢 FollowJointTrajectory,move_group 执行路径全废)或胡萝卜形轨迹(实测 78%,但要对路径做在线 clamp) |
| VLA 后处理安全壳 | 丢失。之前讨论过 servo 的用途之一就是给模型输出兜安全底 |

结论:mujoco 场景保留 servo(`publish_period: 0.05` + omega_max 补偿),直控方案仅在
将来需要满速度且愿意放弃上述安全功能时再考虑。

### 5.6 测试方法备忘

- jog 测速:`/tmp/jog_diag.py <joint> <vel>`(20Hz 发 JointJog,录 /joint_states 与 servo
  输出轨迹;`/tmp` 重启即丢,本节的数字均在 2026-09-20 环境实测)。
- 手动测试前必须确认 servo 已杀干净且 `ros2 topic info .../joint_trajectory` 的
  Publisher count 为 0,否则测量被 servo 的流污染。
- 仿真栈:`nohup ros2 launch so101_moveit_mujoco so101_mujoco.launch.py headless:=true rviz:=false &`;
  servo:`ros2 launch so101_moveit_mujoco so101_moveit_servo.launch.py`。
- 执行器实际力矩观测:`/mujoco_actuators_states` 的 effort 字段(定位饱和全靠它)。

## 6. 待观察项

- `/clock` 由 mujoco_ros2_control 发布,其频率粒度决定 `run_duration: 0.1 (0.01)` 类告警是否出现
  (gazebo 版真因是 /clock 10Hz 量化,同机制);若出现且数值整齐,按无害处理。
- `servo_parameters.yaml` 的 `joint_topic: /joint_states` 默认来源即可;若关节状态延迟大,
  可在 ros2_control xacro 中提高 `so101_joint_state_broadcaster` 的 `update_rate`(当前 50Hz)。

## 7. 操作纪律(沿用 gazebo 版)

- 机械臂异常先 `ros2 control list_controllers` 确认 `so101_arm_controller` active;
  动没动只信 `/joint_states` 的 position。
- CLI 测试发 Twist/JointJog 必须 `stamp: now` 且与仿真时钟同域(CLI 是系统时间,会被当 stale 丢弃),
  测完发一条全零命令刹车;正经测试用带 `use_sim_time:=true` 的节点发。
- xacro/URDF 注释里不能有 ASCII `: `(冒号+空格),写完 `grep ': ' xx.urdf` 自查。
