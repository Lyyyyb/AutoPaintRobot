#!/usr/bin/env python3
import rospy
from track_vehicle import TrackVehicle
from linear_module import LinearModule
from robot_state import RobotState
import globals
from auto_painting_robot import AutoPaintingRobot
# 主程序
if __name__ == '__main__':
    # 创建履带车和直线模组的实例
    autopaintrobot = AutoPaintingRobot()
    vehicle = TrackVehicle()
    linear_module = LinearModule(globals.step_distance, globals.STEP_Y, globals.speed, globals.mode)
    # 设置循环频率
    rate = rospy.Rate(10)  # 10 Hz
    # 主循环
    while not rospy.is_shutdown():        
        # 发布TF变换
        linear_module.publish_transforms()

        # 对于履带车和直线模组，#根据机器人的状态执行相应的操作
        # 主循环中的状态检查和相应的行动
        if AutoPaintingRobot.state == RobotState.NAVIGATING:
            # 如果直线模组的状态是导航（NAVIGATING）状态，
            # 则调用 navigate_to_tree 方法，
            linear_module.navigate_to_tree()

        elif AutoPaintingRobot.state == RobotState.SPRAYING:
            # 如果履带车的状态是喷涂（SPRAYING）状态，
            # 则调用 spray_tree 方法，
            vehicle.spray_tree()
            linear_module.spray_tree()

        else:
            # 如果以上条件都不满足，则执行其他或默认的操作。
            # 在这里，暂时没有定义任何操作（使用 pass 语句）。
            pass

        #调用回调函数
        rospy.spin()        
        # 维持循环频率
        rate.sleep()


