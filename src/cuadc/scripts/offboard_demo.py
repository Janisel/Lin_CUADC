#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from tf.transformations import quaternion_from_euler

class OffboardFSM:
    def __init__(self):
        rospy.init_node('offboard_fsm_node', anonymous=True)

        # 状态变量
        self.current_state = State()
        self.target_pose = PoseStamped()
        
        # 订阅飞控状态
        rospy.Subscriber("mavros/state", State, self.state_cb)
        
        # 发布目标位置
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)

        # 开启一个后台线程，以 20Hz 的频率疯狂发送 target_pose
        # 这是 PX4 维持 Offboard 模式的强制要求（心跳包）
        self.publish_thread = threading.Thread(target=self.setpoint_publisher)
        self.publish_thread.daemon = True
        self.publish_thread.start()

    def state_cb(self, msg):
        self.current_state = msg

    def setpoint_publisher(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.target_pose.header.stamp = rospy.Time.now()
            self.local_pos_pub.publish(self.target_pose)
            rate.sleep()

    def set_target_position(self, x, y, z, yaw_degrees=0):
        """设置目标位置和航向角"""
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        
        # 将角度转换为四元数 (这里只转航向角 Yaw)
        yaw_rad = math.radians(yaw_degrees)
        q = quaternion_from_euler(0, 0, yaw_rad)
        self.target_pose.pose.orientation.x = q[0]
        self.target_pose.pose.orientation.y = q[1]
        self.target_pose.pose.orientation.z = q[2]
        self.target_pose.pose.orientation.w = q[3]

    def wait_for_connection(self):
        rospy.loginfo("等待飞控连接...")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()
        rospy.loginfo("飞控已连接！")

    def run_mission(self):
        self.wait_for_connection()

        # 1. 预热发送几个指令（PX4要求切换Offboard前必须有设定点数据流）
        rospy.loginfo("预热位置指令...")
        self.set_target_position(0, 0, 2) # 初始目标：原地起飞到2米
        rospy.sleep(2.0)

        # 2. 切换 Offboard 模式
        rospy.loginfo("请求切换至 Offboard 模式...")
        if self.set_mode_client(custom_mode="OFFBOARD").mode_sent:
            rospy.loginfo("Offboard 模式切换成功！")
        else:
            rospy.logerr("Offboard 切换失败！程序退出。")
            return

        # 3. 解锁 (Arming)
        rospy.loginfo("请求解锁飞机...")
        if self.arming_client(True).success:
            rospy.loginfo("飞机已解锁！")
        else:
            rospy.logerr("解锁失败！程序退出。")
            return

        # ================== 开始执行状态机逻辑 ==================
        
        # 任务 A：起飞并悬停（到达目标高度2米）
        rospy.loginfo("[状态: 起飞] 正在爬升至2米...")
        rospy.sleep(5.0) # 简单粗暴的延时法，后续我们再换成读取真实坐标判断
        
        # 任务 B：向前飞 5 米 (X轴正方向为前)
        rospy.loginfo("[状态: 航点] 向前飞5米...")
        self.set_target_position(5, 0, 2)
        rospy.sleep(8.0) # 给它8秒钟飞过去
        
        # 任务 C：悬停 5 秒
        rospy.loginfo("[状态: 悬停] 保持当前位置 5 秒...")
        # target_pose 不变，它会自动悬停
        rospy.sleep(5.0)
        
        # 任务 D：降落
        rospy.loginfo("[状态: 降落] 呼叫降落服务...")
        self.set_mode_client(custom_mode="AUTO.LAND")
        # 或者使用: self.land_client(altitude=0, latitude=0, longitude=0, min_pitch=0, yaw=0)
        
        rospy.loginfo("任务执行完毕。")

if __name__ == '__main__':
    try:
        fsm = OffboardFSM()
        fsm.run_mission()
    except rospy.ROSInterruptException:
        pass