#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import curses
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand 
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget

class DroneController:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        rospy.init_node('drone_ctrl_node', anonymous=True)
        
        self.use_ego = False
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.z = 1.5 
        self.yaw = 0.0
        
        # 必须与 FAST-LIO2 的坐标系一致
        self.frame_id = "camera_init" 
        
        # 发布者
        self.pose_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # 订阅规划器
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.ego_cmd_cb)
        
        # MAVROS 服务
        rospy.wait_for_service('/mavros/set_mode')
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.wait_for_service('/mavros/cmd/arming')
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        
        self.auto_initialize()
        
        curses.curs_set(0)
        self.stdscr.nodelay(True)
        self.run()

    def auto_initialize(self):
        rospy.loginfo("Initializing Offboard...")
        for _ in range(20):
            self.publish_pose()
            rospy.sleep(0.05)
        self.arm_srv(True)
        rospy.sleep(0.5)
        self.set_mode_srv(custom_mode='OFFBOARD')

    def ego_cmd_cb(self, msg):
        """核心修正：收到指令即自动激活 AUTO 模式"""
        # 只要开始接收到规划指令，就自动切换模式
        self.use_ego = True 
        
        self.target_pose.pose.position.x = msg.position.x
        self.target_pose.pose.position.y = msg.position.y
        self.target_pose.pose.position.z = msg.position.z
        self.yaw = msg.yaw

    def publish_pose(self):
        q = tft.quaternion_from_euler(0, 0, self.yaw)
        self.target_pose.pose.orientation.x = q[0]
        self.target_pose.pose.orientation.y = q[1]
        self.target_pose.pose.orientation.z = q[2]
        self.target_pose.pose.orientation.w = q[3]
        
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.header.frame_id = self.frame_id
        self.pose_pub.publish(self.target_pose)

    def run(self):
        while not rospy.is_shutdown():
            key = self.stdscr.getch()
            
            # 键盘干预逻辑：任何按键都会强制切回 MANUAL
            if key != -1:
                if key == ord('q'): break
                self.use_ego = False # 停止跟随 EGO
                
                if key == ord('w'): self.target_pose.pose.position.x += 0.2
                if key == ord('s'): self.target_pose.pose.position.x -= 0.2
                if key == ord('a'): self.target_pose.pose.position.y += 0.2
                if key == ord('d'): self.target_pose.pose.position.y -= 0.2
                if key == curses.KEY_UP:    self.target_pose.pose.position.z += 0.1
                if key == curses.KEY_DOWN:  self.target_pose.pose.position.z -= 0.1
                if key == curses.KEY_LEFT:  self.yaw += 0.1
                if key == curses.KEY_RIGHT: self.yaw -= 0.1

            self.publish_pose()
            
            # 实时 UI
            self.stdscr.clear()
            mode_str = "AUTO (EGO-PLANNER)" if self.use_ego else "MANUAL (KEYBOARD)"
            color = curses.A_REVERSE if self.use_ego else curses.A_NORMAL
            
            self.stdscr.addstr(0, 0, "=== OFFBOARD CONTROL HUB ===", curses.A_BOLD)
            self.stdscr.addstr(1, 0, f"MODE: {mode_str}", color)
            self.stdscr.addstr(3, 0, f"Target X: {self.target_pose.pose.position.x:.2f}")
            self.stdscr.addstr(4, 0, f"Target Y: {self.target_pose.pose.position.y:.2f}")
            self.stdscr.addstr(5, 0, f"Target Z: {self.target_pose.pose.position.z:.2f}")
            self.stdscr.addstr(6, 0, f"Yaw     : {self.yaw:.2f}")
            self.stdscr.addstr(8, 0, "HELP: WSAD/Arrows to manual takeover, Q to quit")
            self.stdscr.refresh()
            
            rospy.sleep(0.05)

if __name__ == '__main__':
    try:
        curses.wrapper(DroneController)
    except rospy.ROSInterruptException:
        pass