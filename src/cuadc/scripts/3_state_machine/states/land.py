#降落状态
#使用飞控自带的land模式降落

from states.base_state import BaseState
import rospy

class LandState(BaseState):
    def __init__(self):
        super().__init__("Land")

    def enter(self):
        rospy.loginfo(f"[{self.name}] 触发自动降落模式...")
        # 假设 interface 封装了切换飞行模式的服务
        success = self.interface.set_flight_mode("AUTO.LAND")
        if not success:
            rospy.logerr(f"[{self.name}] 降落模式切换失败！")

    def execute(self):
        # 监听飞控的解锁状态 (Armed)
        # 当飞机触地并自动上锁时，任务彻底完成
        if not self.interface.is_armed():
            rospy.loginfo(f"[{self.name}] 飞机已触地上锁。")
            return True
            
        return False

    def exit(self):
        rospy.loginfo(f"[{self.name}] 降落阶段彻底结束。")