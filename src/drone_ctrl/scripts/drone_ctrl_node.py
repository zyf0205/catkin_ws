#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import curses
import math
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry

class DroneController:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        rospy.init_node('drone_ctrl_node', anonymous=True)

        self.world_frame = "camera_init"

        # ================= 核心状态 =================
        self.use_ego = False
        self.yaw = 0.0

        # 来自 FAST-LIO 的位置（camera_init坐标系）
        self.lio_pos = [0.0, 0.0, 0.0]

        # 来自 MAVROS 的位置（PX4本地坐标系）
        self.mav_pos = [0.0, 0.0, 0.0]
        self.mav_ready = False

        # ✅ 【核心修复】坐标偏移量 = MAVROS位置 - LIO位置
        self.offset = [0.0, 0.0, 0.0]
        self.offset_calibrated = False

        # 手动控制目标（camera_init坐标系下）
        self.manual_target = [0.0, 0.0, 1.0]

        # EGO指令缓存
        self.ego_cmd = None
        self.last_ego_time = rospy.Time(0)

        # ================= 发布器 =================
        self.raw_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local',
            PositionTarget,
            queue_size=10
        )

        # ================= 订阅 =================
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.ego_cmd_cb)
        rospy.Subscriber('/Odometry', Odometry, self.odom_cb)

        # ✅ 【新增】订阅 MAVROS 的本地位置，用来计算偏移量
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pos_cb)

        # ================= MAVROS 服务 =================
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.auto_initialize()

        curses.curs_set(0)
        self.stdscr.nodelay(True)
        self.run()

    # ================= 里程计回调（FAST-LIO） =================
    def odom_cb(self, msg):
        self.lio_pos[0] = msg.pose.pose.position.x
        self.lio_pos[1] = msg.pose.pose.position.y
        self.lio_pos[2] = msg.pose.pose.position.z

    # ================= MAVROS位置回调 =================
    def mavros_pos_cb(self, msg):
        self.mav_pos[0] = msg.pose.position.x
        self.mav_pos[1] = msg.pose.position.y
        self.mav_pos[2] = msg.pose.position.z
        self.mav_ready = True

    # ================= 校准偏移量 =================
    def calibrate_offset(self):
        """计算 MAVROS坐标系 和 camera_init坐标系 之间的偏移"""
        if not self.mav_ready:
            rospy.logwarn("MAVROS position not ready, using zero offset")
            return

        self.offset[0] = self.mav_pos[0] - self.lio_pos[0]
        self.offset[1] = self.mav_pos[1] - self.lio_pos[1]
        self.offset[2] = self.mav_pos[2] - self.lio_pos[2]
        self.offset_calibrated = True

        rospy.loginfo(
            f"Offset calibrated: [{self.offset[0]:.3f}, {self.offset[1]:.3f}, {self.offset[2]:.3f}]"
        )
        rospy.loginfo(
            f"  LIO pos: [{self.lio_pos[0]:.3f}, {self.lio_pos[1]:.3f}, {self.lio_pos[2]:.3f}]"
        )
        rospy.loginfo(
            f"  MAV pos: [{self.mav_pos[0]:.3f}, {self.mav_pos[1]:.3f}, {self.mav_pos[2]:.3f}]"
        )

    # ================= 坐标转换：camera_init → MAVROS本地 =================
    def lio_to_mav(self, x, y, z):
        """将 camera_init 坐标系的值转换为 MAVROS 本地坐标系"""
        return (
            x + self.offset[0],
            y + self.offset[1],
            z + self.offset[2]
        )

    # ================= 初始化 =================
    def auto_initialize(self):
        rospy.loginfo("Waiting for odom and MAVROS...")

        # 等待两个位置源都就绪
        timeout = rospy.Time.now() + rospy.Duration(10.0)
        while not rospy.is_shutdown():
            if self.lio_pos != [0.0, 0.0, 0.0] and self.mav_ready:
                break
            if rospy.Time.now() > timeout:
                rospy.logwarn("Timeout waiting for positions, proceeding anyway")
                break
            rospy.sleep(0.1)

        # ✅ 校准偏移量
        self.calibrate_offset()

        rospy.loginfo("Initializing OFFBOARD...")
        self.manual_target[0] = self.lio_pos[0]
        self.manual_target[1] = self.lio_pos[1]
        self.manual_target[2] = 1.2

        for _ in range(50):
            self.publish_idle()
            rospy.sleep(0.02)

        self.arm_srv(True)
        rospy.sleep(0.5)
        self.set_mode_srv(custom_mode='OFFBOARD')

    # ================= EGO回调 =================
    def ego_cmd_cb(self, msg):
        self.use_ego = True
        self.ego_cmd = msg
        self.last_ego_time = rospy.Time.now()

    # ================= 手动悬停/位置控制 =================
    def publish_idle(self):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        # ✅ 应用偏移量
        mx, my, mz = self.lio_to_mav(
            self.manual_target[0],
            self.manual_target[1],
            self.manual_target[2]
        )
        msg.position.x = mx
        msg.position.y = my
        msg.position.z = mz
        msg.yaw = self.yaw
        self.raw_pub.publish(msg)

    # ================= EGO轨迹追踪 =================
    def publish_ego(self):
        if self.ego_cmd is None:
            return

        if (rospy.Time.now() - self.last_ego_time).to_sec() > 0.5:
            self.use_ego = False
            self.manual_target = [self.lio_pos[0], self.lio_pos[1], self.lio_pos[2]]
            return

        cmd = self.ego_cmd
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # 位置 + 速度前馈
        msg.type_mask = (
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        # ✅ 位置应用偏移量
        px, py, pz = self.lio_to_mav(
            cmd.position.x,
            cmd.position.y,
            cmd.position.z
        )
        msg.position.x = px
        msg.position.y = py
        msg.position.z = pz

        # ✅ 速度不需要偏移（速度是矢量，与原点无关）
        msg.velocity.x = cmd.velocity.x
        msg.velocity.y = cmd.velocity.y
        msg.velocity.z = cmd.velocity.z

        msg.yaw = cmd.yaw
        self.raw_pub.publish(msg)

    # ================= 键盘控制 =================
    def handle_keyboard(self, key):
        if key == ord('q'): return False

        if key == ord('e'):
            self.use_ego = True

        if key == ord('m'):
            self.use_ego = False
            self.manual_target = [self.lio_pos[0], self.lio_pos[1], self.lio_pos[2]]

        # ✅ 按C重新校准偏移量（飞行中也可以用）
        if key == ord('c'):
            self.calibrate_offset()

        if not self.use_ego:
            step = 0.2
            if key == ord('w'): self.manual_target[0] += step
            if key == ord('s'): self.manual_target[0] -= step
            if key == ord('a'): self.manual_target[1] += step
            if key == ord('d'): self.manual_target[1] -= step
            if key == curses.KEY_UP:    self.manual_target[2] += 0.1
            if key == curses.KEY_DOWN:  self.manual_target[2] -= 0.1
            if key == curses.KEY_LEFT:  self.yaw += 0.15
            if key == curses.KEY_RIGHT: self.yaw -= 0.15
        return True

    # ================= 主循环 =================
    def run(self):
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            key = self.stdscr.getch()
            if key != -1:
                if not self.handle_keyboard(key):
                    break

            if self.use_ego:
                self.publish_ego()
            else:
                self.publish_idle()

            self.draw_ui()
            rate.sleep()

    # ================= UI =================
    def draw_ui(self):
        self.stdscr.clear()
        mode_str = "AUTO (EGO TRACKING)" if self.use_ego else "MANUAL (HOVER)"
        color = curses.A_REVERSE if self.use_ego else curses.A_NORMAL

        self.stdscr.addstr(0, 0, "=== EGO-PLANNER OFFBOARD BRIDGE ===", curses.A_BOLD)
        self.stdscr.addstr(1, 0, f"MODE: {mode_str}", color)

        # 位置信息
        self.stdscr.addstr(3, 0, f"LIO Pos:  [{self.lio_pos[0]:.2f}, {self.lio_pos[1]:.2f}, {self.lio_pos[2]:.2f}]")
        self.stdscr.addstr(4, 0, f"MAV Pos:  [{self.mav_pos[0]:.2f}, {self.mav_pos[1]:.2f}, {self.mav_pos[2]:.2f}]")

        # ✅ 显示偏移量
        cal_str = "YES" if self.offset_calibrated else "NO"
        self.stdscr.addstr(5, 0, f"Offset:   [{self.offset[0]:.3f}, {self.offset[1]:.3f}, {self.offset[2]:.3f}] (Cal:{cal_str})")

        if self.use_ego and self.ego_cmd:
            px, py, pz = self.lio_to_mav(
                self.ego_cmd.position.x,
                self.ego_cmd.position.y,
                self.ego_cmd.position.z
            )
            self.stdscr.addstr(7, 0,  f"EGO Target (LIO): [{self.ego_cmd.position.x:.2f}, {self.ego_cmd.position.y:.2f}, {self.ego_cmd.position.z:.2f}]")
            self.stdscr.addstr(8, 0,  f"EGO Target (MAV): [{px:.2f}, {py:.2f}, {pz:.2f}]")
            self.stdscr.addstr(9, 0,  f"EGO Vel:          [{self.ego_cmd.velocity.x:.2f}, {self.ego_cmd.velocity.y:.2f}, {self.ego_cmd.velocity.z:.2f}]")
        else:
            mx, my, mz = self.lio_to_mav(
                self.manual_target[0],
                self.manual_target[1],
                self.manual_target[2]
            )
            self.stdscr.addstr(7, 0, f"Manual Target (LIO): [{self.manual_target[0]:.2f}, {self.manual_target[1]:.2f}, {self.manual_target[2]:.2f}]")
            self.stdscr.addstr(8, 0, f"Manual Target (MAV): [{mx:.2f}, {my:.2f}, {mz:.2f}]")
            self.stdscr.addstr(9, 0, f"Yaw: {self.yaw:.2f}")

        self.stdscr.addstr(11, 0, "W/S/A/D: XY | Up/Dn: Z | Left/Right: Yaw")
        self.stdscr.addstr(12, 0, "E: EGO | M: Manual | C: Re-calibrate | Q: Quit")
        self.stdscr.refresh()

if __name__ == '__main__':
    try:
        curses.wrapper(DroneController)
    except rospy.ROSInterruptException:
        pass