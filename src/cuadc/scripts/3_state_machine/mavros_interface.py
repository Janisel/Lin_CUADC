#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.srv import ParamSet
from mavros_msgs.msg import ParamValue
from mavros_msgs.msg import PositionTarget

class MavrosInterface:
    def __init__(self):
        # 内部状态变量
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        
        # 订阅飞控状态与当前位置
        rospy.Subscriber("mavros/state", State, self.state_cb)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        # 全局地速设置 (默认 2.0 m/s)
        self.current_cruise_speed = 2.0 
        
        self.target_raw = PositionTarget()
        # 设定坐标系为局部 NED (MAVROS会自动把我们的ENU转换过去)
        self.target_raw.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # 核心掩码：忽略加速度(64|128|256)，忽略角速度(2048)。只使用位置(X,Y,Z)、速度(Vx,Vy,Vz)和偏航角(Yaw)
        self.target_raw.type_mask = 64 | 128 | 256 | 2048 
        
        # 改为发布 raw 话题
        self.raw_pos_pub = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

        # 参数设置客户端
        self.param_set_client = rospy.ServiceProxy("mavros/param/set", ParamSet)

        # 核心：启动后台心跳线程，维持 Offboard 模式不掉线
        self.heartbeat_thread = threading.Thread(target=self.heartbeat_publisher)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def heartbeat_publisher(self):
        """以 20Hz 疯狂向飞控发送 target_raw"""
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.target_raw.header.stamp = rospy.Time.now()
            self.raw_pos_pub.publish(self.target_raw)
            rate.sleep()

    # ================= 供状态类调用的简洁接口 =================
    
    def set_target_raw(self, x, y, z, vx, vy, vz, yaw=0.0):
        """同时下发位置与前馈速度"""
        self.target_raw.position.x = x
        self.target_raw.position.y = y
        self.target_raw.position.z = z
        self.target_raw.velocity.x = vx
        self.target_raw.velocity.y = vy
        self.target_raw.velocity.z = vz
        self.target_raw.yaw = yaw

    def set_target_position(self, x, y, z):
        """兼容老代码的接口（用于起飞、悬停等不需要速度前馈的状态）"""
        # 调用新的 raw 接口，将前馈速度强制设为 0
        self.set_target_raw(x, y, z, vx=0.0, vy=0.0, vz=0.0, yaw=0.0)

    def get_current_position(self):
        """返回当前的 (x, y, z) 元组"""
        return (self.current_pose.pose.position.x, 
                self.current_pose.pose.position.y, 
                self.current_pose.pose.position.z)

    def is_armed(self):
        """检查是否处于解锁状态"""
        return self.current_state.armed

    def set_flight_mode(self, custom_mode):
        """切换飞行模式（如 OFFBOARD, AUTO.LAND）"""
        return self.set_mode_client(custom_mode=custom_mode).mode_sent
