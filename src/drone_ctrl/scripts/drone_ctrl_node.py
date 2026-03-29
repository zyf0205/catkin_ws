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

        # ================= 配置参数 =================
        # 必须与 ego_planner launch 里的 frame_id 一致
        self.world_frame = "camera_init" 
        
        # ================= 核心状态 =================
        self.use_ego = False
        self.yaw = 0.0
        self.current_pos = [0.0, 0.0, 0.0]
        
        # 手动控制目标
        self.manual_target = [0.0, 0.0, 1.0]

        # 当前ego指令缓存
        self.ego_cmd = None
        self.last_ego_time = rospy.Time(0)

        # ================= 发布器 =================
        # 使用 setpoint_raw 是最稳妥的方式
        self.raw_pub = rospy.Publisher(
            '/mavros/setpoint_raw/local',
            PositionTarget,
            queue_size=10
        )

        # ================= 订阅 =================
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.ego_cmd_cb)
        rospy.Subscriber('/Odometry', Odometry, self.odom_cb)

        # ================= MAVROS 服务 =================
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        # 初始化飞控状态
        self.auto_initialize()

        # Curses UI 设置
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        self.run()

    def odom_cb(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        self.current_pos[2] = msg.pose.pose.position.z

    def auto_initialize(self):
        rospy.loginfo("Waiting for odom...")
        # 等待里程计数据，确保起飞位置正确
        while self.current_pos == [0.0, 0.0, 0.0] and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        rospy.loginfo("Initializing OFFBOARD...")
        self.manual_target[0] = self.current_pos[0]
        self.manual_target[1] = self.current_pos[1]
        self.manual_target[2] = 1.2 # 设定一个安全的初始高度

        # MAVROS 要求在切模式前必须先有指令流
        for _ in range(50):
            self.publish_idle()
            rospy.sleep(0.02)
        
        self.arm_srv(True)
        rospy.sleep(0.5)
        self.set_mode_srv(custom_mode='OFFBOARD')

    def ego_cmd_cb(self, msg):
        # 收到规划指令时，自动切换到 EGO 模式（也可手动按 E 键）
        self.use_ego = True
        self.ego_cmd = msg
        self.last_ego_time = rospy.Time.now()

    # ================= 模式1：手动悬停/位置控制 =================
    def publish_idle(self):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED # 实际上 MAVROS 内部会处理 ENU->NED 转换

        # 掩码：忽略速度、忽略加速度、忽略角速度
        msg.type_mask = (
            PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        msg.position.x = self.manual_target[0]
        msg.position.y = self.manual_target[1]
        msg.position.z = self.manual_target[2]
        msg.yaw = self.yaw
        self.raw_pub.publish(msg)

    # ================= 模式2：EGO 轨迹追踪 =================
    def publish_ego(self):
        if self.ego_cmd is None:
            return

        # 安全检查：如果规划器停止发消息超过0.5秒，切回手动悬停
        if (rospy.Time.now() - self.last_ego_time).to_sec() > 0.5:
            self.use_ego = False
            self.manual_target = [self.current_pos[0], self.current_pos[1], self.current_pos[2]]
            return

        cmd = self.ego_cmd
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.world_frame
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # 核心修复：使用【位置 + 速度】组合控制，忽略加速度（减少噪声抖动）
        # 如果追求极高动态性能，可以取消 IGNORE_AFX 等，但通常不需要
        msg.type_mask = (
            PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        # 直接透传规划器的输出值
        msg.position.x = cmd.position.x
        msg.position.y = cmd.position.y
        msg.position.z = cmd.position.z

        msg.velocity.x = cmd.velocity.x
        msg.velocity.y = cmd.velocity.y
        msg.velocity.z = cmd.velocity.z

        msg.yaw = cmd.yaw
        
        self.raw_pub.publish(msg)

    def handle_keyboard(self, key):
        if key == ord('q'): return False
        
        # 模式切换
        if key == ord('e'): self.use_ego = True
        if key == ord('m'): 
            self.use_ego = False
            self.manual_target = [self.current_pos[0], self.current_pos[1], self.current_pos[2]]

        # 手动微调目标点
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

    def run(self):
        rate = rospy.Rate(50) # 提高到 50Hz 追踪更平滑

        while not rospy.is_shutdown():
            key = self.stdscr.getch()
            if key != -1:
                if not self.handle_keyboard(key):
                    break

            if self.use_ego:
                self.publish_ego()
            else:
                self.publish_idle()

            # UI 显示
            self.draw_ui()
            rate.sleep()

    def draw_ui(self):
        self.stdscr.clear()
        mode_str = "AUTO (EGO TRACKING)" if self.use_ego else "MANUAL (HOVER)"
        color = curses.A_REVERSE if self.use_ego else curses.A_NORMAL

        self.stdscr.addstr(0, 0, "=== EGO-PLANNER OFFBOARD BRIDGE ===", curses.A_BOLD)
        self.stdscr.addstr(1, 0, f"MODE: {mode_str}", color)
        self.stdscr.addstr(3, 0, f"Current Pos: [{self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f}]")
        
        if self.use_ego and self.ego_cmd:
            self.stdscr.addstr(4, 0, f"Planner Target: [{self.ego_cmd.position.x:.2f}, {self.ego_cmd.position.y:.2f}, {self.ego_cmd.position.z:.2f}]")
            self.stdscr.addstr(5, 0, f"Planner Vel:    [{self.ego_cmd.velocity.x:.2f}, {self.ego_cmd.velocity.y:.2f}, {self.ego_cmd.velocity.z:.2f}]")
        else:
            self.stdscr.addstr(4, 0, f"Manual Target:  [{self.manual_target[0]:.2f}, {self.manual_target[1]:.2f}, {self.manual_target[2]:.2f}]")
            self.stdscr.addstr(5, 0, f"Manual Yaw:     {self.yaw:.2f}")

        self.stdscr.addstr(8, 0, "CONTROLS: E-EGO Mode | M-Manual | W/S/A/D-XY | Up/Dn-Z | Left/Right-Yaw | Q-Quit")
        self.stdscr.refresh()

if __name__ == '__main__':
    try:
        curses.wrapper(DroneController)
    except rospy.ROSInterruptException:
        pass