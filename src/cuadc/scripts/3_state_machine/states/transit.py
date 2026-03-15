import rospy
import math
from states.base_state import BaseState

class TransitState(BaseState):
    def __init__(self, target_x, target_y, target_z, speed=2.0, tolerance=0.2):
        super().__init__("Transit")
        # 最终目的地
        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        # 期望的总地速 (m/s)
        self.speed = speed          
        self.tolerance = tolerance 
        
        # 虚拟目标点（用于平滑牵引飞机）
        self.virtual_x = 0.0
        self.virtual_y = 0.0
        self.virtual_z = 0.0
        
        # 用于计算循环时间间隔 (dt)
        self.last_time = None

    def enter(self):
        # 状态开始时，获取飞机当前真实的物理位置，将其作为虚拟目标点的起点
        curr_x, curr_y, curr_z = self.interface.get_current_position()
        self.virtual_x = curr_x
        self.virtual_y = curr_y
        self.virtual_z = curr_z
        
        self.last_time = rospy.Time.now()
        rospy.loginfo(f"[{self.name}] 飞往 (X:{self.target_x}, Y:{self.target_y}, Z:{self.target_z}) | 匀速设定: {self.speed} m/s")

    def execute(self):
        # 1. 计算两次执行之间真实经过的时间 dt (大约为 0.05秒)
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # 防护机制：如果电脑卡顿导致 dt 过大，限制最大步长防止飞机突然暴冲
        if dt > 0.1:
            dt = 0.05
            
        # 2. 获取当前真实的物理位置
        curr_x, curr_y, curr_z = self.interface.get_current_position()
        
        # 3. 计算【真实位置】到【最终目标点】的误差，用于判定任务是否真正完成
        real_dx = self.target_x - curr_x
        real_dy = self.target_y - curr_y
        real_dz = self.target_z - curr_z
        real_distance = math.sqrt(real_dx**2 + real_dy**2 + real_dz**2)
        
        if real_distance <= self.tolerance:
            rospy.loginfo(f"[{self.name}] 航点到达，当前误差: {real_distance:.2f} 米")
            return True
            
        # 4. 计算【虚拟目标点】到【最终目标点】的向量
        v_dx = self.target_x - self.virtual_x
        v_dy = self.target_y - self.virtual_y
        v_dz = self.target_z - self.virtual_z
        v_distance = math.sqrt(v_dx**2 + v_dy**2 + v_dz**2)
        
        # 5. 速度分解：将总地速拆解为 Vx, Vy, Vz 分量 (方向直指最终目标)
        if v_distance > 0.01:
            vx = (v_dx / v_distance) * self.speed
            vy = (v_dy / v_distance) * self.speed
            vz = (v_dz / v_distance) * self.speed
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
        
        # 6. 移动虚拟目标点 (上一帧位置 + 速度 × 时间)
        step_distance = self.speed * dt # 这一帧理论上应该前进的距离
        
        # 如果这一步的前进距离超过了剩余距离，说明虚拟点到站了，直接钉在终点
        if step_distance >= v_distance:
            self.virtual_x = self.target_x
            self.virtual_y = self.target_y
            self.virtual_z = self.target_z
            # 虚拟点到站后，前馈速度清零，让 P 控制器收敛最后的真实物理误差
            vx, vy, vz = 0.0, 0.0, 0.0
        else:
            self.virtual_x += vx * dt
            self.virtual_y += vy * dt
            self.virtual_z += vz * dt
        
        # 7. 同时下发【前方的虚拟点】与【带有方向的速度分量】
        self.interface.set_target_raw(self.virtual_x, self.virtual_y, self.virtual_z, vx, vy, vz, yaw=0.0)
        
        return False

    def exit(self):
        # 退出该状态时，界面清空残余速度指令，确保悬停状态不出错
        pass