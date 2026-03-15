import rospy
from states.base_state import BaseState

class TakeoffState(BaseState):
    def __init__(self, target_alt):
        super().__init__("Takeoff")
        self.target_alt = target_alt
        # 新增：用于记录起飞瞬间的真实XY坐标
        self.start_x = 0.0
        self.start_y = 0.0

    def enter(self):
        # 1. 在状态初始化瞬间，获取真实的物理位置
        curr_x, curr_y, _ = self.interface.get_current_position()
        
        # 2. 锁定XY坐标
        self.start_x = curr_x
        self.start_y = curr_y
        
        rospy.loginfo(f"[{self.name}] 锁定起飞原点 (X:{self.start_x:.2f}, Y:{self.start_y:.2f})")
        rospy.loginfo(f"[{self.name}] 开始垂直起飞，目标高度: {self.target_alt} 米")

    def execute(self):
        # 3. 持续发送锁定的XY坐标和目标高度Z（保证绝对垂直升空）
        self.interface.set_target_position(self.start_x, self.start_y, self.target_alt)
        
        # 4. 获取当前高度并判断误差
        _, _, current_z = self.interface.get_current_position()
        error = abs(self.target_alt - current_z)
        
        if error < 0.2:
            rospy.loginfo(f"[{self.name}] 到达目标高度。")
            return True
            
        return False

    def exit(self):
        rospy.loginfo(f"[{self.name}] 起飞阶段结束。")