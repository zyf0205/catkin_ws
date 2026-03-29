#!/bin/bash

rosclean purge -y

# 1. 启动 PX4 SITL 仿真底盘
gnome-terminal --tab --title="PX4" -- bash -c "roslaunch px4 mavros_posix_sitl.launch; exec bash"
sleep 3

# 2. 启动 FAST-LIO2 (提供高精度里程计和点云)
gnome-terminal --tab --title="FAST-LIO" -- bash -c "roslaunch fast_lio mapping_mid360.launch use_sim_time:=true; exec bash"
sleep 2

# 3. 启动 TF/Odom 桥接 (将 Odom 喂给 PX4 形成闭环)
gnome-terminal --tab --title="Bridge" -- bash -c "rosrun px4_fastlio_bridge odom_to_vision.py; exec bash"
sleep 2

# 4. 启动 EGO-Planner (你的避障大脑)
# 注意：确认这里的包名(ego_planner)和 launch 名是否与你实际的一致
gnome-terminal --tab --title="EGO-Plan" -- bash -c "roslaunch ego_planner simple_run.launch; exec bash"
sleep 2

# 5. 打开 QGC 监控机体状态 (路径请换成你自己的)
gnome-terminal --tab --title="QGC" -- bash -c "/home/zyf/qgc_v4.3/QGroundControl.AppImage; exec bash"
sleep 2

# 6. 启动控制节点 (等待手动回车确认)
# 这个节点负责接收 EGO 的轨迹并发送给 PX4 (OFFBOARD 模式)
gnome-terminal --tab --title="Control" -- bash -c "echo '等待各节点初始化完毕...'; read -p '确认无误后，按回车启动控制节点:'; rosrun drone_ctrl drone_ctrl_node.py; exec bash"
