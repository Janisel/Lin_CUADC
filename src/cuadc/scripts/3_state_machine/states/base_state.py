#规定了每个状态必须拥有 enter, execute, exit 三个标准生命周期函数。
class BaseState:
    def __init__(self, name):
        self.name = name          # 状态名称，用于日志打印
        self.interface = None     # 预留接口，用于调用底层的无人机控制函数

    def enter(self):
        """进入该状态时执行一次"""
        pass

    def execute(self):
        """主循环中以 20Hz 疯狂执行，返回 True 表示任务完成，返回 False 表示继续"""
        return False

    def exit(self):
        """退出该状态时执行一次"""
        pass