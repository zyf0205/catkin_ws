#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
import numpy as np
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry

def get_yaw(q):
    """从四元数提取偏航角"""
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

def wrap_angle(angle):
    """将角度限制在 -pi 到 pi 之间"""
    return (angle + np.pi) % (2 * np.pi) - np.pi

class DroneController:
    def __init__(self):
        rospy.init_node('drone_ctrl_node')
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.2)

        self.lio_pos = np.zeros(3)
        self.lio_yaw = 0.0
        self.ego_cmd = None
        self.last_ego_time = rospy.Time(0)

        self.raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        rospy.Subscriber('/Odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.ego_cb)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.lio_pos = np.array([p.x, p.y, p.z])
        self.lio_yaw = get_yaw(msg.pose.pose.orientation)

    def ego_cb(self, msg):
        self.ego_cmd, self.last_ego_time = msg, rospy.Time.now()

    def send_pos(self, pos, yaw, vel=None):
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED, yaw=yaw)
        msg.header.stamp = rospy.Time.now()
        msg.position.x, msg.position.y, msg.position.z = pos
        if vel is not None:
            msg.velocity.x, msg.velocity.y, msg.velocity.z = vel
            msg.type_mask = 2496 # 忽略加速度、偏航率
        else:
            msg.type_mask = 2552 # 绝对静止悬停
        self.raw_pub.publish(msg)

    def run(self):
        rospy.loginfo("Waiting for LIO and MAVROS...")
        try:
            odom = rospy.wait_for_message('/Odometry', Odometry, timeout=15)
            mav = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=15)
        except rospy.ROSException:
            return

        self.odom_cb(odom)
        mav_pos = np.array([mav.pose.position.x, mav.pose.position.y, mav.pose.position.z])
        mav_yaw = get_yaw(mav.pose.orientation)
        
        # ✅ 核心修复：同时计算 XYZ 偏移 和 YAW 角度偏移
        offset = mav_pos - self.lio_pos
        yaw_offset = wrap_angle(mav_yaw - self.lio_yaw) 
        
        target_pos = self.lio_pos + [0, 0, self.takeoff_height] + offset
        target_yaw = wrap_angle(self.lio_yaw + yaw_offset)

        rospy.loginfo(f"XYZ Offset: {np.round(offset, 2)} | Yaw Offset: {yaw_offset:.2f} rad")

        for _ in range(100):
            self.send_pos(target_pos, target_yaw)
            rospy.sleep(0.02)
            
        rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)(True)
        rospy.ServiceProxy('/mavros/set_mode', SetMode)(custom_mode='OFFBOARD')
        rospy.loginfo("OFFBOARD armed")

        rate = rospy.Rate(50)
        cnt = 0

        while not rospy.is_shutdown():
            is_ego = self.ego_cmd and (rospy.Time.now() - self.last_ego_time).to_sec() < 0.5
            
            if is_ego:
                c = self.ego_cmd
                pos = np.array([c.position.x, c.position.y, c.position.z]) + offset
                vel = np.array([c.velocity.x, c.velocity.y, c.velocity.z])
                
                # ✅ 目标位置和机头朝向都加上 offset
                target_pos = pos
                target_yaw = wrap_angle(c.yaw + yaw_offset)

                if np.linalg.norm(vel) < 0.05:
                    self.send_pos(target_pos, target_yaw)
                else:
                    self.send_pos(target_pos, target_yaw, vel)
            else:
                self.send_pos(target_pos, target_yaw)

            if cnt % 50 == 0:
                rospy.loginfo(f"Target XYZ:[{target_pos[0]:.2f}, {target_pos[1]:.2f}, {target_pos[2]:.2f}] Yaw:{target_yaw:.2f}")
            cnt += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        DroneController().run()
    except rospy.ROSInterruptException:
        pass