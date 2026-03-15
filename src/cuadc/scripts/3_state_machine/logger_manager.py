#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import math
import rospy
import threading
from datetime import datetime

# 导入必要的 ROS 消息类型
from mavros_msgs.msg import State, StatusText
from geometry_msgs.msg import PoseStamped, TwistStamped
from rosgraph_msgs.msg import Log

class LoggerManager:
    def __init__(self):
        # ================= 1. 自动生成续存的日志文件路径 =================
        self.log_dir = os.path.expanduser("~/桌面/cuadc_ws/log")
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        # 查找当前文件夹下最大的编号
        existing_files = [f for f in os.listdir(self.log_dir) if f.endswith('.csv') and f[:4].isdigit()]
        if not existing_files:
            next_id = 1
        else:
            ids = [int(f[:4]) for f in existing_files]
            next_id = max(ids) + 1
            
        self.prefix = f"{next_id:04d}"
        self.csv_path = os.path.join(self.log_dir, f"{self.prefix}.csv")
        self.txt_path = os.path.join(self.log_dir, f"{self.prefix}.txt")

        # 打开文件（以追加模式写入TXT，写入模式创建CSV）
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8')
        self.txt_file = open(self.txt_path, 'a', encoding='utf-8')
        
        self.csv_writer = csv.writer(self.csv_file)
        # 写入 CSV 表头
        self.csv_writer.writerow(['Time_s', 'X_m', 'Y_m', 'Z_m', 'GroundSpeed_m_s', 'Yaw_deg'])
        
        # 记录初始化信息到 TXT
        self.write_txt("SYSTEM", "日志模块初始化成功，等待切换至 OFFBOARD 模式...")

        # ================= 2. 数据暂存变量 =================
        self.is_recording = False
        self.start_time = 0.0
        
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_z = 0.0
        self.curr_ground_speed = 0.0
        self.curr_yaw = 0.0

        # ================= 3. 订阅真实物理数据与日志话题 =================
        # 订阅飞控状态（用于触发记录）
        rospy.Subscriber("mavros/state", State, self.state_cb)
        
        # 订阅真实的 ENU 坐标与姿态
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.pose_cb)
        
        # 订阅真实的三维速度分量
        rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.vel_cb)
        
        # 【核心技巧1】订阅 PX4 飞控内部的警告与报错文字
        rospy.Subscriber("mavros/statustext/recv", StatusText, self.px4_status_cb)
        
        # 【核心技巧2】订阅整个 ROS 系统的文本输出，拦截你在状态机里写的 loginfo
        rospy.Subscriber("/rosout", Log, self.ros_log_cb)

        # ================= 4. 开启 CSV 高频写入线程 (10Hz) =================
        self.record_thread = threading.Thread(target=self.csv_record_loop)
        self.record_thread.daemon = True
        self.record_thread.start()

    # ------------------ 回调函数处理区 ------------------

    def write_txt(self, source, message):
        """格式化写入 TXT 日志"""
        time_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_line = f"[{time_str}] [{source}] {message}\n"
        self.txt_file.write(log_line)
        self.txt_file.flush() # 强制立刻刷入硬盘，防止炸机断电丢失

    def state_cb(self, msg):
        """检测是否切入 Offboard 以启动记录"""
        if msg.mode == "OFFBOARD" and not self.is_recording:
            self.is_recording = True
            self.start_time = rospy.Time.now().to_sec()
            self.write_txt("EVENT", "飞机已切换至 OFFBOARD 模式，CSV 数据开始录制！")

    def pose_cb(self, msg):
        """更新真实位置与偏航角"""
        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y
        self.curr_z = msg.pose.position.z
        
        # 手动将四元数转为 Yaw 角（弧度转度数）
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        self.curr_yaw = math.degrees(yaw_rad)

    def vel_cb(self, msg):
        """更新真实的水平地速"""
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        # 计算水平面上的合速度大小
        self.curr_ground_speed = math.sqrt(vx**2 + vy**2)

    def px4_status_cb(self, msg):
        """拦截飞控底层发出的警告（如电量低、GPS丢失、拒绝解锁等）"""
        # PX4 严重级别 (Severity): 0=Emergency, 3=Error, 4=Warning, 6=Info
        if msg.severity <= 4:
            self.write_txt("PX4_WARNING", msg.text)
        else:
            self.write_txt("PX4_INFO", msg.text)

    def ros_log_cb(self, msg):
        """拦截状态机执行的日志"""
        # 只捕获我们自己写的 main_fsm_node 发出的日志，忽略其他无关节点的杂音
        if msg.name == "/main_fsm_node":
            self.write_txt("FSM_STATE", msg.msg)

    # ------------------ 录制主循环 ------------------

    def csv_record_loop(self):
        """以 10Hz 的频率将当前真实数据写入 CSV"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_recording:
                current_time = rospy.Time.now().to_sec() - self.start_time
                self.csv_writer.writerow([
                    f"{current_time:.2f}", 
                    f"{self.curr_x:.3f}", 
                    f"{self.curr_y:.3f}", 
                    f"{self.curr_z:.3f}", 
                    f"{self.curr_ground_speed:.3f}", 
                    f"{self.curr_yaw:.2f}"
                ])
                self.csv_file.flush() # 强制刷入硬盘
            rate.sleep()

    def close(self):
        """节点关闭时安全释放文件"""
        self.write_txt("SYSTEM", "程序结束，日志记录关闭。")
        self.csv_file.close()
        self.txt_file.close()