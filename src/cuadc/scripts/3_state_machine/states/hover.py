#悬停状态

from states.base_state import BaseState
import rospy

class HoverState(BaseState):
    def __init__(self, hover_time):
        super().__init__("Hover")
        self.hover_time = hover_time
        self.start_time = None
        self.hover_x = 0
        self.hover_y = 0
        self.hover_z = 0

    def enter(self):
        self.start_time = rospy.Time.now()
        # 锁定进入该状态瞬间的位置，作为悬停锚点
        current_pos = self.interface.get_current_position()
        self.hover_x = current_pos[0]
        self.hover_y = current_pos[1]
        self.hover_z = current_pos[2]
        
        rospy.loginfo(f"[{self.name}] 开始原地悬停，倒计时 {self.hover_time} 秒...")

    def execute(self):
        # 持续发送悬停锚点坐标，防止飞机漂移
        self.interface.set_target_position(self.hover_x, self.hover_y, self.hover_z)
        
        # 计算流逝时间
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        if elapsed_time >= self.hover_time:
            rospy.loginfo(f"[{self.name}] 悬停时间到。")
            return True
            
        return False

    def exit(self):
        rospy.loginfo(f"[{self.name}] 悬停阶段结束。")