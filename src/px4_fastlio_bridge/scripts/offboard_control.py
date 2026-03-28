#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node('offboard_node', anonymous=True)

    rospy.Subscriber("/mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    rate = rospy.Rate(20) # 20Hz

    # 1. 等待飞控心跳连接
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    rospy.loginfo("MAVROS Connected!")

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2.0

    # 2. 预填充缓冲区的延时大幅缩短：只发 20 个点（1秒钟）足够骗过飞控了
    rospy.loginfo("Sending initial setpoints...")
    for i in range(20):
        if rospy.is_shutdown():
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    rospy.loginfo("Attempting to switch to OFFBOARD and ARM...")
    
    while not rospy.is_shutdown():
        # 如果不是 Offboard，且距离上次请求超过 1 秒（原来是5秒，太长了）
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            if set_mode_client.call(offb_set_mode).mode_sent == True:
                rospy.loginfo("OFFBOARD enabled")
            last_req = rospy.Time.now()
        
        # 已经是 Offboard，但还没解锁，且距离上次请求超过 1 秒
        elif current_state.mode == "OFFBOARD" and not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(1.0):
            if arming_client.call(arm_cmd).success == True:
                rospy.loginfo("Vehicle armed, taking off!")
            last_req = rospy.Time.now()

        # 无论如何，死循环必须以20Hz发送目标点
        local_pos_pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass