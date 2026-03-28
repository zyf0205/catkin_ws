#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class OdomToVision:
    def __init__(self):
        rospy.init_node('odom_to_vision_node')
        
        # 发布给 MAVROS 的视觉位姿话题
        self.vision_pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        
        # 订阅 Fast-LIO2 的里程计话题
        # 注意：检查你的 Fast-LIO2 话题名到底是 /Odometry 还是其他的
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback)
        
        rospy.loginfo("Bridge Node Started: Forwarding /Odometry to /mavros/vision_pose/pose")

    def odom_callback(self, msg):
        vision_msg = PoseStamped()
        
        # 1. 对齐时间戳 (非常关键，PX4 EKF2 严格依赖时间戳，建议用当前ROS时间覆盖)
        vision_msg.header.stamp = rospy.Time.now()
        vision_msg.header.frame_id = "map" # 对应 MAVROS 的全局固定系
        
        # 2. 搬运位置 (Position)
        vision_msg.pose.position.x = msg.pose.pose.position.x
        vision_msg.pose.position.y = msg.pose.pose.position.y
        vision_msg.pose.position.z = msg.pose.pose.position.z
        
        # 3. 搬运姿态 (Orientation - 四元数)
        vision_msg.pose.orientation.x = msg.pose.pose.orientation.x
        vision_msg.pose.orientation.y = msg.pose.pose.orientation.y
        vision_msg.pose.orientation.z = msg.pose.pose.orientation.z
        vision_msg.pose.orientation.w = msg.pose.pose.orientation.w
        
        # 发送给 PX4
        self.vision_pub.publish(vision_msg)

if __name__ == '__main__':
    try:
        node = OdomToVision()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass