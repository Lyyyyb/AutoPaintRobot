import rospy
import math
import serial
from auto_painting_robot import AutoPaintingRobot
from robot_state import RobotState
import tf
import globals

# #步进电机一步滑台移动的距离
# STEP_DISTANCE = globals.get_step_distance()  # 例如，每步0.1米
# distance_threshold = globals.get_distance_threshold()  # 例如，1米的阈值
# STEPS = globals.get_steps #丝杠上下移动的步数

# 直线模组，继承自AutoPaintingRobot
class LinearModule(AutoPaintingRobot):
    def __init__(self):
        super().__init__()

        # 初始化串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

    # 创建控制直线模组的串口指令
    def create_serial_command(self, x, y, speed, mode):
        command = f"move X{x} Y{y} speed{speed} mode{mode}"
        return "Module " + command

    # 发送控制直线模组的串口指令
    def send_serial_command(self, command):
        try:
            self.serial_port.write(command.encode())
        except serial.SerialException as e:
            rospy.logerr("Linear module serial communication error: %s", e)


    def convert_to_module_frame(self, tree_position, listener):
        # 这个方法用于将树木位置从激光雷达坐标系转换到模块坐标系

        try:
            # 等待激光雷达坐标系（/lidar_frame）到模块坐标系（/module_frame）的转换关系变得可用
            # 此处等待时间设置为4秒，可根据实际需要调整
            self.listener.waitForTransform("/module_frame", "/lidar_frame", rospy.Time(0), rospy.Duration(4.0))

            # 将树木位置从激光雷达坐标系转换到模块坐标系
            # tree_position 应该是一个包含树木位置的geometry_msgs/PointStamped消息
            tree_position_module_frame = self.listener.transformPoint("/module_frame", tree_position)

            # 返回转换后的树木位置
            return tree_position_module_frame

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            # 如果在转换过程中出现异常（例如，找不到坐标系、连接问题或时间外推异常）
            # 则记录错误信息并返回None
            rospy.logerr("TF转换异常: %s", e)
            return None

        
    def calculate_movement_for_module(self, tree_position_module_frame):
        # 假设直线模组的起点为(0,0,0)，计算滑台需要移动的距离
        movement_distance = tree_position_module_frame.x  # 根据实际情况可能需要调整
        return movement_distance
    
    def move_and_operate_spray_claw(self, STEP_DISTANCE):
        # 检测树木的位置，使用来自激光雷达的数据
        tree_position = self.detect_tree_from_lidar(self.lidar_data)

        # 将检测到的树木位置从激光雷达坐标系转换到模块坐标系
        tree_position_module_frame = self.convert_to_module_frame(tree_position, self.listener)

        if tree_position_module_frame is not None:
            # 计算从当前位置到目标位置的移动距离
            movement_distance = self.calculate_movement_for_module(tree_position_module_frame)

            # 根据移动距离计算出步进电机需要走的步数
            steps = self.calculate_steps_for_motor(movement_distance, STEP_DISTANCE)

            # 创建并发送串口指令以控制机器人移动相应的步数
            move_command = self.create_serial_command(steps, steps, 1000, 1)
            self.send_serial_command(move_command)

            # 控制喷爪的操作
            self.open_spray_claw()


    
    def calculate_steps_for_motor(self, movement_distance, step_distance):
        # 计算步进电机需要运动的步数
        steps = int(movement_distance / step_distance)  # 可能需要进行取整操作
        return steps
    
    def open_spray_claw(self):
        # 发送指令以张开喷爪
        open_command = "OPEN_CLAW"
        self.send_serial_command(open_command)

    def close_spray_claw(self):
        # 发送指令以闭合喷爪
        close_command = "CLOSE_CLAW"
        self.send_serial_command(close_command)

    # 实现具体的喷涂树木逻辑
    def spray_tree(self):
        serial_data = self.read_serial_data()

        if serial_data == "OK":
            if self.state_machine.state == 2:
                """
                调整车的朝向后，使用三维雷达获取树的坐标，通过TF坐标转换成相对于X轴直线模组最左侧的坐标，
                通过坐标信息计算出X轴直线模组的滑台需要移动多少距离，进而得到X轴直线模组的步进电机需要运动的步数，
                转换成控制指令，通过串口发送给下位机。

                调整X轴丝杠，正对树，张开喷爪
                """
                #控制丝杠和喷爪
                self.move_and_operate_spray_claw(globals.get_step_distance())
                #更新状态量
                self.state_machine.update_state(3)

            elif self.state_machine.state == 4:
                """
                控制水泵喷漆，控制Z轴直线模组的滑台，使Y轴直线模组上下移动对树均匀喷漆。
                控制Y轴直线模组末端的喷爪张开

                喷爪闭合，控制Z轴丝杠，使Y轴丝杠上下运动，同时控制水泵喷水，喷涂结束，喷爪张开
                """
                # 发送指令以闭合喷涂装置的喷爪
                self.close_spray_claw()
                # 创建并发送串口指令以控制机器人移动相应的步数
                # 假设：我们将 X 和 Y 设为步数，速度和模式为预设值
                move_command = self.create_serial_command(0, globals.get_steps(), 1000, 1)
                self.send_serial_command(move_command)
                #更新状态量
                self.state_machine.update_state(5)

            elif self.state_machine.state == 6:
                """
                喷爪闭合
                """
                #闭合喷爪
                self.close_spray_claw()
                #更新状态量
                self.state_machine.update_state(7)

        elif serial_data == "ERROR":
            # 错误处理逻辑
            rospy.logwarn("Serial communication error in LinearModule during spraying")
            # 可以在此处添加更多的错误处理逻辑