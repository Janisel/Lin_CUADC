import rospy
import math
from states.base_state import BaseState

class RelativeTransitState(BaseState):
    def __init__(self, dx, dy, dz, speed=None, tolerance=0.2):
        super().__init__("RelTransit")
        # 传入的是相对于当前位置的偏移量 (ENU坐标系)
        self.dx = dx
        self.dy = dy
        self.dz = dz
        
        # 真正的绝对目标点（将在 enter 中动态计算）
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0
        
        self.speed = speed
        self.tolerance = tolerance
        
        # 刹车减速参数（复用平滑刹车逻辑）
        self.decel_radius = 2.0
        self.min_speed = 0.3
        
        self.virtual_x = 0.0
        self.virtual_y = 0.0
        self.virtual_z = 0.0
        self.last_time = None

        # 新增：目标是否已经计算过的记忆锁
        self.is_target_calculated = False

    def enter(self):
        # 1. 获取进入状态瞬间的真实物理位置
        curr_x, curr_y, curr_z = self.interface.get_current_position()
        
        # 新增判断：只有在第一次进入时，才计算绝对目标点
        if not self.is_target_calculated:

            # 2. 动态计算出真正的绝对目标点
            self.target_x = curr_x + self.dx
            self.target_y = curr_y + self.dy
            self.target_z = curr_z + self.dz
            self.is_target_calculated = True # 锁死，下次恢复时不再重新计算

            rospy.loginfo(f"[{self.name}] 首次计算相对偏移 (dX:{self.dx}, dY:{self.dy}, dZ:{self.dz})")
            rospy.loginfo(f"[{self.name}] 锁定绝对坐标 -> X:{self.target_x:.2f}, Y:{self.target_y:.2f}, Z:{self.target_z:.2f}")
        else:
            rospy.loginfo(f"[{self.name}] 从中断中恢复，继续飞往原定目标 -> X:{self.target_x:.2f}, Y:{self.target_y:.2f}, Z:{self.target_z:.2f}")
        # 【极其重要】：无论是否是第一次，虚拟牵引点（胡萝卜）必须每次都重置在机身当前位置

        # 3. 初始化虚拟目标点
        self.virtual_x = curr_x
        self.virtual_y = curr_y
        self.virtual_z = curr_z
        
        self.last_time = rospy.Time.now()
        use_speed = self.speed if self.speed is not None else self.interface.current_cruise_speed
        
        rospy.loginfo(f"[{self.name}] 相对偏移指令 (dX:{self.dx}, dY:{self.dy}, dZ:{self.dz})")
        rospy.loginfo(f"[{self.name}] 实际飞往绝对坐标 -> X:{self.target_x:.2f}, Y:{self.target_y:.2f}, Z:{self.target_z:.2f} | 地速: {use_speed} m/s")

    def execute(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        if dt > 0.1:
            dt = 0.05
            
        curr_x, curr_y, curr_z = self.interface.get_current_position()
        
        real_dx = self.target_x - curr_x
        real_dy = self.target_y - curr_y
        real_dz = self.target_z - curr_z
        real_distance = math.sqrt(real_dx**2 + real_dy**2 + real_dz**2)
        
        if real_distance <= self.tolerance:
            rospy.loginfo(f"[{self.name}] 相对航点到达，当前误差: {real_distance:.2f} 米")
            return True
            
        max_speed = self.speed if self.speed is not None else self.interface.current_cruise_speed
        
        # 动态平滑减速
        if real_distance <= self.decel_radius:
            current_speed = max_speed * (real_distance / self.decel_radius)
            current_speed = max(self.min_speed, current_speed)
        else:
            current_speed = max_speed

        v_dx = self.target_x - self.virtual_x
        v_dy = self.target_y - self.virtual_y
        v_dz = self.target_z - self.virtual_z
        v_distance = math.sqrt(v_dx**2 + v_dy**2 + v_dz**2)
        
        if v_distance > 0.01:
            vx = (v_dx / v_distance) * current_speed
            vy = (v_dy / v_distance) * current_speed
            vz = (v_dz / v_distance) * current_speed
        else:
            vx, vy, vz = 0.0, 0.0, 0.0
        
        step_distance = current_speed * dt
        
        if step_distance >= v_distance:
            self.virtual_x = self.target_x
            self.virtual_y = self.target_y
            self.virtual_z = self.target_z
            vx, vy, vz = 0.0, 0.0, 0.0
        else:
            self.virtual_x += vx * dt
            self.virtual_y += vy * dt
            self.virtual_z += vz * dt
        
        self.interface.set_target_raw(self.virtual_x, self.virtual_y, self.virtual_z, vx, vy, vz, yaw=0.0)
        
        return False

    def exit(self):
        pass