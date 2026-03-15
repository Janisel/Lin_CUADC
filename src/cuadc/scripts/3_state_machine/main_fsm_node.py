#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from mavros_interface import MavrosInterface
from logger_manager import LoggerManager

# 导入你写的各个状态积木
from states.takeoff import TakeoffState
from states.hover import HoverState
from states.land import LandState
from states.transit import TransitState
from states.relative_transit import RelativeTransitState

class FSMEngine:
    def __init__(self):
        rospy.init_node('main_fsm_node', anonymous=True)
        
        # === 新增：启动日志记录黑匣子 ===
        self.logger = LoggerManager()
        # =================================

        # 实例化底层通信接口
        self.interface = MavrosInterface()

        rospy.loginfo("等待飞控连接...")
        while not rospy.is_shutdown() and not self.interface.current_state.connected:
            rospy.sleep(0.1)
        rospy.loginfo("飞控已连接！")

        # ================= 拼装你的飞行计划 =================
        self.mission_queue = [
            TakeoffState(target_alt=5.0),  # 起飞到 5 米
            HoverState(hover_time=3.0),    # 悬停 3 秒

            RelativeTransitState(dx=30.0, dy=0.0, dz=0.0, speed=3.0),
            #TransitState(30.0,0.0,5.0, speed=3.0),
            HoverState(hover_time=3.0), 

            # 【相对飞行】在到达 (30, 0, 5) 后，向正东（X轴方向）再推进 3 米，向正北（Y轴方向）平移 10 米
            RelativeTransitState(dx=3.0, dy=10.0, dz=0.0, speed=1.0),
            HoverState(hover_time=3.0), 
            LandState()                    # 降落
        ]

        # 【核心逻辑】：把 interface 注入到每一个状态中，让他们有控制飞机的能力
        for state in self.mission_queue:
            state.interface = self.interface

    def run(self):
        # 1. Offboard 预热与解锁准备
        rospy.loginfo("正在预热并请求 Offboard 控制权...")
        self.interface.set_target_position(0, 0, 0)
        rospy.sleep(3.0)
        
        self.interface.set_flight_mode("OFFBOARD")
        self.interface.arming_client(True)

        if not self.mission_queue:
            rospy.logwarn("任务队列为空！")
            return

        # 2. 弹出第一个任务，执行 enter 初始化
        current_state = self.mission_queue.pop(0)
        current_state.enter()

        # 3. 开启 20Hz 状态机主循环
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            # ================= 新增：全局看门狗 (Watchdog) =================
            # 检查是否意外退出了 OFFBOARD 模式（例如：遥控器抢夺，或 GPS 丢失导致飞控主动切出）
            # 注意：必须排除 Land 状态，因为降落状态我们本来就会主动切到 AUTO.LAND 模式
            if self.interface.current_state.mode != "OFFBOARD" and current_state.name != "Land":
                rospy.logerr(f"【安全拦截】OFFBOARD 控制权丢失！当前飞行模式: {self.interface.current_state.mode}")
                rospy.logwarn("状态机已挂起，停止发送前馈速度，等待人为恢复 OFFBOARD 模式...")

                # 死循环阻塞，直到安全员重新将模式切回 OFFBOARD
                while not rospy.is_shutdown() and self.interface.current_state.mode != "OFFBOARD":
                    rate.sleep()

                rospy.loginfo("【恢复】OFFBOARD 控制权已重新获取！")
                rospy.loginfo("正在根据飞机当前新位置，重新规划平滑轨迹...")
                # 【极其关键的一步】：必须重新调用 enter()！
                # 这会让 TransitState 重新获取飞机当前的新位置，并把虚拟目标点重置到机身内部。
                # 彻底消除重获控制权瞬间的暴冲现象。
                current_state.enter()
            # ===============================================================

            # 疯狂调用当前状态的 execute
            is_done = current_state.execute()

            # 如果当前状态返回 True (任务完成)
            if is_done:
                current_state.exit()  # 执行该状态的收尾工作
                
                # 检查队列里是否还有下一个任务
                if len(self.mission_queue) > 0:
                    current_state = self.mission_queue.pop(0)
                    current_state.enter()
                else:
                    rospy.loginfo("=== 所有任务执行完毕 ===")
                    break
                    
            rate.sleep()

if __name__ == '__main__':
    try:
        engine = FSMEngine()
        engine.run()
    except rospy.ROSInterruptException:
        pass