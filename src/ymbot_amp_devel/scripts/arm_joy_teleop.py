#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手臂摇杆步长控制节点 - arm_joy_teleop.py

通过摇杆按键/轴的步长累积方式，控制双臂的 shoulder_pitch 和 shoulder_roll 关节。
发布 shoulder_joint_commands (std_msgs/Float64MultiArray)，
data 顺序：[左肩pitch, 左肩roll, 右肩pitch, 右肩roll]

控制映射 (默认，可通过 ROS 参数修改):
  按钮 4 (L1) 按下时使能手臂控制（死区按钮）
  D-Pad 上/下 (轴 7, 正/负):  左肩 pitch +/-
  D-Pad 左/右 (轴 6, 正/负):  左肩 roll  +/-
  右摇杆 上/下 (轴 3, 正/负): 右肩 pitch +/-
  右摇杆 左/右 (轴 2, 正/负): 右肩 roll  +/-
  按钮 1 (圆圈/B):             重置所有关节到默认位置
"""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class ArmJoyTeleop:
    def __init__(self):
        rospy.init_node('arm_joy_teleop')

        # ── 可调参数（可通过 rosparam 修改） ─────────────────────────────
        self.deadman_button     = rospy.get_param('~deadman_button', 4)      # L1
        self.reset_button       = rospy.get_param('~reset_button', 1)        # 圆圈/B

        # 左肩：D-Pad 上下控制 pitch，D-Pad 左右控制 roll
        self.l_pitch_axis       = rospy.get_param('~l_pitch_axis', 7)        # D-Pad 上下
        self.l_roll_axis        = rospy.get_param('~l_roll_axis',  6)        # D-Pad 左右

        # 右肩：右摇杆上下控制 pitch，右摇杆左右控制 roll
        self.r_pitch_axis       = rospy.get_param('~r_pitch_axis', 3)        # 右摇杆上下
        self.r_roll_axis        = rospy.get_param('~r_roll_axis',  2)        # 右摇杆左右

        self.step_size          = rospy.get_param('~step_size', 0.05)        # 每次按下的步长 (rad)
        self.axis_threshold     = rospy.get_param('~axis_threshold', 0.5)    # 轴触发阈值

        # 默认位置和限位
        self.default_l_pitch    = rospy.get_param('~default_l_pitch',  0.35)
        self.default_l_roll     = rospy.get_param('~default_l_roll',   0.0)
        self.default_r_pitch    = rospy.get_param('~default_r_pitch', -0.35)
        self.default_r_roll     = rospy.get_param('~default_r_roll',   0.0)

        self.l_pitch_min        = rospy.get_param('~l_pitch_min', -1.5)
        self.l_pitch_max        = rospy.get_param('~l_pitch_max',  1.5)
        self.l_roll_min         = rospy.get_param('~l_roll_min',  -1.0)
        self.l_roll_max         = rospy.get_param('~l_roll_max',   1.0)
        self.r_pitch_min        = rospy.get_param('~r_pitch_min', -1.5)
        self.r_pitch_max        = rospy.get_param('~r_pitch_max',  1.5)
        self.r_roll_min         = rospy.get_param('~r_roll_min',  -1.0)
        self.r_roll_max         = rospy.get_param('~r_roll_max',   1.0)

        publish_rate            = rospy.get_param('~publish_rate', 20.0)     # Hz

        # ── 内部状态 ────────────────────────────────────────────────────
        self.l_pitch = self.default_l_pitch
        self.l_roll  = self.default_l_roll
        self.r_pitch = self.default_r_pitch
        self.r_roll  = self.default_r_roll

        # 记录上次轴状态（用于边沿检测，避免持续触发）
        self._prev = {
            'l_pitch': 0.0,
            'l_roll':  0.0,
            'r_pitch': 0.0,
            'r_roll':  0.0,
        }
        self._prev_reset = False

        # ── ROS 接口 ─────────────────────────────────────────────────────
        self.pub = rospy.Publisher(
            'shoulder_joint_commands',
            Float64MultiArray,
            queue_size=1
        )
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        rospy.Timer(rospy.Duration(1.0 / publish_rate), self.timer_callback)

        rospy.loginfo("=== Arm Joy Teleop Node Started ===")
        rospy.loginfo("Deadman button : %d (L1)", self.deadman_button)
        rospy.loginfo("Reset button   : %d (Circle/B)", self.reset_button)
        rospy.loginfo("L pitch axis   : %d  | L roll axis : %d", self.l_pitch_axis, self.l_roll_axis)
        rospy.loginfo("R pitch axis   : %d  | R roll axis : %d", self.r_pitch_axis, self.r_roll_axis)
        rospy.loginfo("Step size      : %.3f rad", self.step_size)
        rospy.loginfo("Default pos    : L[%.2f, %.2f]  R[%.2f, %.2f]",
                      self.l_pitch, self.l_roll, self.r_pitch, self.r_roll)

    # ── 辅助函数 ─────────────────────────────────────────────────────────

    def _rising_edge(self, key, current_val):
        """检测轴从阈值以内到超出阈值的上升沿（正向或负向）。
        返回 +1（正向触发）、-1（负向触发）或 0（无触发）。"""
        prev = self._prev[key]
        result = 0
        if current_val > self.axis_threshold and prev <= self.axis_threshold:
            result = +1
        elif current_val < -self.axis_threshold and prev >= -self.axis_threshold:
            result = -1
        self._prev[key] = current_val
        return result

    def _clamp(self, val, lo, hi):
        return max(lo, min(hi, val))

    # ── 回调函数 ─────────────────────────────────────────────────────────

    def joy_callback(self, msg):
        n_axes    = len(msg.axes)
        n_buttons = len(msg.buttons)

        # 检查死区按钮
        deadman_ok = (n_buttons > self.deadman_button and
                      msg.buttons[self.deadman_button] == 1)

        # 重置按钮（不需要死区）
        reset_pressed = (n_buttons > self.reset_button and
                         msg.buttons[self.reset_button] == 1)
        if reset_pressed and not self._prev_reset:
            self.l_pitch = self.default_l_pitch
            self.l_roll  = self.default_l_roll
            self.r_pitch = self.default_r_pitch
            self.r_roll  = self.default_r_roll
            rospy.loginfo("Arm positions RESET to default: L[%.2f, %.2f]  R[%.2f, %.2f]",
                          self.l_pitch, self.l_roll, self.r_pitch, self.r_roll)
        self._prev_reset = reset_pressed

        if not deadman_ok:
            # 死区未按下时也要刷新轴状态，防止松开后立即触发
            if n_axes > self.l_pitch_axis:
                self._prev['l_pitch'] = msg.axes[self.l_pitch_axis]
            if n_axes > self.l_roll_axis:
                self._prev['l_roll']  = msg.axes[self.l_roll_axis]
            if n_axes > self.r_pitch_axis:
                self._prev['r_pitch'] = msg.axes[self.r_pitch_axis]
            if n_axes > self.r_roll_axis:
                self._prev['r_roll']  = msg.axes[self.r_roll_axis]
            return

        # ── 左肩 pitch ────────────────────────────────────────────────
        if n_axes > self.l_pitch_axis:
            edge = self._rising_edge('l_pitch', msg.axes[self.l_pitch_axis])
            if edge != 0:
                self.l_pitch = self._clamp(
                    self.l_pitch + edge * self.step_size,
                    self.l_pitch_min, self.l_pitch_max
                )
                rospy.loginfo("L pitch -> %.3f", self.l_pitch)

        # ── 左肩 roll ─────────────────────────────────────────────────
        if n_axes > self.l_roll_axis:
            edge = self._rising_edge('l_roll', msg.axes[self.l_roll_axis])
            if edge != 0:
                self.l_roll = self._clamp(
                    self.l_roll + edge * self.step_size,
                    self.l_roll_min, self.l_roll_max
                )
                rospy.loginfo("L roll  -> %.3f", self.l_roll)

        # ── 右肩 pitch ────────────────────────────────────────────────
        if n_axes > self.r_pitch_axis:
            edge = self._rising_edge('r_pitch', msg.axes[self.r_pitch_axis])
            if edge != 0:
                self.r_pitch = self._clamp(
                    self.r_pitch + edge * self.step_size,
                    self.r_pitch_min, self.r_pitch_max
                )
                rospy.loginfo("R pitch -> %.3f", self.r_pitch)

        # ── 右肩 roll ─────────────────────────────────────────────────
        if n_axes > self.r_roll_axis:
            edge = self._rising_edge('r_roll', msg.axes[self.r_roll_axis])
            if edge != 0:
                self.r_roll = self._clamp(
                    self.r_roll + edge * self.step_size,
                    self.r_roll_min, self.r_roll_max
                )
                rospy.loginfo("R roll  -> %.3f", self.r_roll)

    def timer_callback(self, event):
        """以固定频率持续发布当前目标位置。"""
        msg = Float64MultiArray()
        msg.data = [self.l_pitch, self.l_roll, self.r_pitch, self.r_roll]
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        node = ArmJoyTeleop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
