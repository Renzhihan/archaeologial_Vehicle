# Archaeologial_Vehicle

#### 介绍
基于ROS的智能考古土方运输系统开源代码

#### 开发环境
Ubuntu18.04+ROS Melodic

#### 采用传感器
煜禾森FR-07底盘+Intel第11代NUC+DWM1000－UWB基站与标签+思岚A1激光雷达+维特智能IMU

#### 软件架构
软件架构说明

├─ackermann_msgs-master 
├─archaeology ---------------------- // 主板<br>
├─archaeology_sub ------------------ // 副板<br>
├─d435i ---------------------------- // 深度相机<br>
├─imu ------------------------------ // imu<br>
├─navigation-noetic-devel ---------- // 导航包<br>
├─roborts_common <br>
├─roborts_decision ----------------- // 决策<br>
├─roborts_msgs --------------------- // ros消息<br>
├─robot_setup_tf ------------------- // tf设置<br>
├─rplidar_ros-master --------------- // 思岚雷达<br>
├─serial --------------------------- // 串口<br>
├─socket_server -------------------- // socket通信<br>
├─stm32_master --------------------- // 主板<br>
├─stm32_sub ------------------------ // 副板<br>
├─teb_local_planner_tutorials ------ // teb规划<br>
├─test_package --------------------- // 测试<br>
├─yhs_can_control ------------------ // 底盘控制<br>
└─yhs_can_msgs --------------------- // 底盘消息<br>

#### 使用说明

1.  roslaunch teb_local_planner_tutorials test.launch
2.  rosrun socket_server socket_server_node
    //[注] 服务器运行后 手机端小程序根据IP连接到服务器 在小程序端发出指令后 服务器会向决策模块发送对应指令（格式见msg中appdata）
3.  xxxx



