#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
import numpy as np
from geometry_msgs.msg import PoseStamped
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import PositionTarget
from nav_msgs.msg import Odometry

class DroneController:
    def __init__(self):
        rospy.init_node('drone_ctrl_node')
        self.takeoff_height = rospy.get_param("~takeoff_height", 1.2)

        # 状态变量
        self.lio_pos = np.zeros(3)
        self.lio_yaw = 0.0
        self.ego_cmd = None
        self.last_ego_time = rospy.Time(0)

        # 发布与订阅
        self.raw_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        rospy.Subscriber('/Odometry', Odometry, self.odom_cb)
        rospy.Subscriber('/planning/pos_cmd', PositionCommand, self.ego_cb)

    def odom_cb(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.lio_pos = np.array([p.x, p.y, p.z])
        self.lio_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2 + q.z**2))

    def ego_cb(self, msg):
        self.ego_cmd, self.last_ego_time = msg, rospy.Time.now()

    def send_pos(self, pos, yaw, vel=None):
        """发送位置指令（MAVROS坐标系）"""
        msg = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED, yaw=yaw)
        msg.header.stamp = rospy.Time.now()
        msg.position.x, msg.position.y, msg.position.z = pos
        if vel is not None:
            msg.velocity.x, msg.velocity.y, msg.velocity.z = vel
            msg.type_mask = 2496 # 忽略加速度、偏航率
        else:
            msg.type_mask = 2552 # 忽略速度、加速度、偏航率
        self.raw_pub.publish(msg)

    def run(self):
        rospy.loginfo("Waiting for LIO and MAVROS...")
        try: # 阻塞等待单帧数据进行初始化，省去循环标志位
            odom = rospy.wait_for_message('/Odometry', Odometry, timeout=15)
            mav = rospy.wait_for_message('/mavros/local_position/pose', PoseStamped, timeout=15)
        except rospy.ROSException:
            rospy.logerr("Timeout waiting for sensor data.")
            return

        # 1. 计算坐标偏移 (只算一次，无需持续订阅mav_pos)
        self.odom_cb(odom) # 手动调一次更新初始位置
        mav_pos = np.array([mav.pose.position.x, mav.pose.position.y, mav.pose.position.z])
        offset = mav_pos - self.lio_pos
        
        # 2. 设置起飞点
        hover_pos = self.lio_pos + [0, 0, self.takeoff_height]
        hover_yaw = self.lio_yaw
        rospy.loginfo(f"Offset: {np.round(offset, 2)} | Takeoff Target: {np.round(hover_pos, 2)}")

        # 3. 预发Setpoint与解锁 (连发100次，耗时2秒)
        for _ in range(100):
            self.send_pos(hover_pos + offset, hover_yaw)
            rospy.sleep(0.02)
            
        rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)(True)
        rospy.ServiceProxy('/mavros/set_mode', SetMode)(custom_mode='OFFBOARD')
        rospy.loginfo("OFFBOARD armed")

        # 4. 主循环
        rate = rospy.Rate(50)
        was_ego, cnt = False, 0

        while not rospy.is_shutdown():
            # 判断EGO指令是否超时 (<0.5s)
            is_ego = self.ego_cmd and (rospy.Time.now() - self.last_ego_time).to_sec() < 0.5
            
            if is_ego:
                c = self.ego_cmd
                pos = np.array([c.position.x, c.position.y, c.position.z]) + offset
                vel = np.array([c.velocity.x, c.velocity.y, c.velocity.z])
                self.send_pos(pos, c.yaw, vel)
                was_ego = True
            else:
                if was_ego: # 刚停下，锁定当前位置
                    hover_pos, hover_yaw = self.lio_pos.copy(), self.lio_yaw
                    rospy.loginfo(f"EGO stopped, hover locked at: {np.round(hover_pos, 2)}")
                    was_ego = False
                self.send_pos(hover_pos + offset, hover_yaw)

            # 打印日志 (1Hz)
            if cnt % 50 == 0:
                rospy.loginfo(f"EGO:{is_ego} | LIO:{np.round(self.lio_pos, 2)} | Hover:{np.round(hover_pos, 2)}")
            cnt += 1
            rate.sleep()

if __name__ == '__main__':
    try:
        DroneController().run()
    except rospy.ROSInterruptException:
        pass