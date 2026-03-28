#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import curses
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool

class DroneController:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        rospy.init_node('drone_ctrl_node', anonymous=True)
        
        # 初始目标姿态
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 2.0 # 初始起飞高度
        self.yaw = 0.0 
        
        # 定义发布者和订阅者
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_cb)
        
        # 服务连接
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        # 初始化序列
        self.auto_initialize()
        
        curses.curs_set(0) # 隐藏光标
        self.stdscr.nodelay(True) # 非阻塞输入
        self.run()

    def auto_initialize(self):
        rospy.loginfo("Initializing Offboard sequence...")
        for _ in range(20):
            self.publish_pose()
            rospy.sleep(0.05)
        self.arm_srv(True)
        rospy.sleep(0.5)
        self.set_mode_srv(custom_mode='OFFBOARD')
        rospy.loginfo("Auto-initialization complete.")

    def rviz_cb(self, msg):
        """RViz 2D Nav Goal 回调 - 修复：忽略 RViz 发来的 Z=0"""
        self.target_pose.pose.position.x = msg.pose.position.x
        self.target_pose.pose.position.y = msg.pose.position.y
        # 安全检查
        self.target_pose.pose.position.z = max(0.5, msg.pose.position.z)
            
        q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.yaw = tft.euler_from_quaternion(q)[2]
        rospy.loginfo("RViz Goal received!")

    def publish_pose(self):
        q = tft.quaternion_from_euler(0, 0, self.yaw)
        self.target_pose.pose.orientation.x, self.target_pose.pose.orientation.y, \
        self.target_pose.pose.orientation.z, self.target_pose.pose.orientation.w = q
        
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = "map"
        self.pose_pub.publish(self.target_pose)

    def run(self):
        while not rospy.is_shutdown():
            key = self.stdscr.getch()
            
            # --- 键盘控制 ---
            if key == ord('w'): self.target_pose.pose.position.x += 0.5
            if key == ord('s'): self.target_pose.pose.position.x -= 0.5
            if key == ord('a'): self.target_pose.pose.position.y += 0.5
            if key == ord('d'): self.target_pose.pose.position.y -= 0.5
            if key == curses.KEY_UP:    self.target_pose.pose.position.z += 0.2
            if key == curses.KEY_DOWN:
                if self.target_pose.pose.position.z > 0.2: 
                    self.target_pose.pose.position.z -= 0.2
            if key == curses.KEY_LEFT:  self.yaw += 0.1
            if key == curses.KEY_RIGHT: self.yaw -= 0.1
            if key == ord('q'): break 

            self.publish_pose()
            
            # --- 核心：UI 刷新逻辑 ---
            self.stdscr.clear()
            self.stdscr.addstr(0, 0, "=== AUTO-OFFBOARD CONTROL ===")
            self.stdscr.addstr(1, 0, "WSAD: Move XY | UP/DOWN: Z | LEFT/RIGHT: Yaw")
            self.stdscr.addstr(2, 0, "Press 'Q' to quit")
            self.stdscr.addstr(4, 0, f"Target: X:{self.target_pose.pose.position.x:.1f} Y:{self.target_pose.pose.position.y:.1f} Z:{self.target_pose.pose.position.z:.1f} Yaw:{self.yaw:.2f}")
            self.stdscr.refresh() # <--- 这一行是显示的关键
            
            rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        curses.wrapper(DroneController)
    except rospy.ROSInterruptException:
        pass