# 定义机器人的可能状态
class RobotState:
    NAVIGATING = 1  # 导航状态
    SPRAYING = 2    # 喷涂状态

# 状态机类，用于管理喷涂过程中的状态
class SprayingStateMachine:
    def __init__(self):
        self.state = 7  # 初始状态

    def update_state(self, new_state):
        # 更新状态机的状态
        self.state = new_state
