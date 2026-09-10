#!/usr/bin/env python3
"""SO101 MuJoCo 手动控制台（机械臂 + 夹爪）。

用法:
    ros2 run so101_moveit_mujoco manual_control.py

前提:so101_mujoco.launch.py 已启动，三个控制器 active。
世界里固定有一个红色小圆柱障碍物（MJCF so101_scene.xml 写死，
位置与 gazebo 版相同，不能被移动或删除）。

主菜单:
    1) 移动机械臂   —— 子菜单:选关节输入目标角度,或开/关夹爪;
                       关闭夹爪用位置指令(关节下限 -0.17=闭合,上限 1.74=张开),
                       每次关闭后按夹爪关节角度检测一次(没合到下限=夹到东西),会提示
    2) 查看状态     —— 打印各关节当前角、夹爪状态
    q) 退出

输入时用 Ctrl+C 可随时退出整个脚本。
"""
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
# mujoco 版夹爪是 JointGroupPositionController(位置指令),与 gazebo 版的力控不同
GRIP_OPEN_POS = 1.0     # 张开(上限 1.74533)
GRIP_CLOSE_POS = -0.17  # 闭合(下限 -0.17453,略高于下限保证全闭)
GRIP_CLOSED_LIMIT = -0.1745  # 关节下限:空夹闭合时钳口最终停的位置


class ManualControl(Node):

    @staticmethod
    def read_line(prompt):
        """从 fd 0 无缓冲读一行(避免 Python stdin 缓冲把后续按键预读吞掉)。"""
        sys.stdout.write(prompt)
        sys.stdout.flush()
        buf = b''
        while True:
            ch = os.read(0, 1)
            if not ch:
                raise EOFError
            if ch == b'\n':
                break
            buf += ch
        return buf.decode(errors='replace').strip()

    def __init__(self):
        super().__init__('so101_manual_control')
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/so101_gripper_controller/commands', 1)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/so101_arm_controller/joint_trajectory', 1)
        self._joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

    # ---------- 订阅回调 ----------
    def _on_joint_state(self, msg):
        self._joint_state = msg

    # ---------- 状态读取 ----------
    def refresh(self, seconds=0.3):
        """泵一会儿事件,让订阅数据更新。"""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def joint_positions(self, refresh_s=0.3):
        """返回 {关节名: 位置},未收齐返回 None。"""
        self.refresh(refresh_s)
        if not self._joint_state:
            return None
        names = list(self._joint_state.name)
        if not all(j in names for j in ARM_JOINTS + ['gripper']):
            return None
        return {j: self._joint_state.position[names.index(j)]
                for j in ARM_JOINTS + ['gripper']}

    # ---------- 动作 ----------
    def send_joint(self, joint, target, duration=2.0):
        """单关节移动到目标角,其余关节保持在当前值。"""
        pos = self.joint_positions()
        if pos is None:
            print('!! 读不到关节状态')
            return False
        joints = ARM_JOINTS if joint in ARM_JOINTS else [joint]
        msg = JointTrajectory()
        msg.joint_names = joints
        point = JointTrajectoryPoint()
        point.positions = [target if j == joint else pos[j] for j in joints]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        msg.points = [point]
        self.traj_pub.publish(msg)
        time.sleep(duration + 0.5)
        return True

    def send_gripper(self, value):
        msg = Float64MultiArray()
        msg.data = [value]
        self.gripper_pub.publish(msg)

    def check_grasped(self):
        """用夹爪关节角度判断夹到东西没有。

        空夹时钳口会一路闭合到关节下限(-0.1745);钳口之间有障碍物时,
        关节停在比下限明显更开的位置。
        """
        pos = self.joint_positions()
        if pos is None:
            return False
        return pos['gripper'] > GRIP_CLOSED_LIMIT + 0.05

    # ---------- 菜单 ----------
    def show_status(self):
        pos = self.joint_positions()
        if pos:
            for j in ARM_JOINTS + ['gripper']:
                print(f'  {j:14s} = {pos[j]:+.3f} rad')
            grip_state = '闭合' if pos['gripper'] < 0.5 else '张开'
            print(f'  夹爪: {grip_state}')
        print('  障碍物: 固定红色圆柱 (0.303, 0.009, 0.031),在 MJCF 场景里写死')

    def move_arm_menu(self):
        while rclpy.ok():
            pos = self.joint_positions()
            if pos is None:
                print('!! 读不到关节状态,回主菜单')
                return
            print('\n-- 移动机械臂 --')
            for i, j in enumerate(ARM_JOINTS, 1):
                print(f'  {i}) {j:14s} 当前 {pos[j]:+.3f} rad')
            grip_state = '闭合' if pos['gripper'] < 0.5 else '张开'
            print(f'  o) 打开夹爪    c) 关闭夹爪(位置 {GRIP_CLOSE_POS},夹到会提示)   当前:{grip_state}')
            print('  b) 返回上一级')
            choice = self.read_line('选择: ').lower()
            try:
                if choice == 'b':
                    return
                if choice == 'o':
                    self.send_gripper(GRIP_OPEN_POS)
                    print('OK 夹爪已打开')
                    continue
                if choice == 'c':
                    self.send_gripper(GRIP_CLOSE_POS)
                    print(f'OK 夹爪闭合指令已发({GRIP_CLOSE_POS}),等待夹稳...')
                    time.sleep(2.0)
                    if self.check_grasped():
                        print('*** 夹到东西了! ***')
                    else:
                        print('.. 没夹到东西(空合)')
                    continue
                idx = int(choice) - 1
                if not (0 <= idx < len(ARM_JOINTS)):
                    print('!! 无效选择')
                    continue
                self.adjust_joint(ARM_JOINTS[idx])
            except (ValueError, EOFError):
                print('!! 无效输入')

    # ---------- 关节调整 ----------
    def adjust_joint(self, joint):
        """输入目标角度,发给 arm_controller 执行。"""
        pos = self.joint_positions()
        raw = self.read_line(f'目标角度 rad(当前 {pos[joint]:+.3f},回车取消): ')
        if raw:
            try:
                self.send_joint(joint, float(raw), 2.0)
            except ValueError:
                print('!! 不是数字')


def main():
    rclpy.init()
    node = ManualControl()
    try:
        print('SO101 MuJoCo 手动控制台(前提:so101_mujoco.launch.py 已启动,控制器 active)')
        while rclpy.ok():
            print('\n== 主菜单 ==')
            print('  1) 移动机械臂')
            print('  2) 查看状态')
            print('  q) 退出')
            choice = node.read_line('选择: ').lower()
            if choice == 'q':
                break
            if choice == '1':
                node.move_arm_menu()
            elif choice == '2':
                node.show_status()
            else:
                print('!! 无效选择')
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
