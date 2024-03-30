import rospy
import math
import serial
from auto_painting_robot import AutoPaintingRobot
from kalman_filter import KalmanFilter
import numpy as np
import globals 
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

# #步进电机一步滑台移动的距离
# STEP_DISTANCE = globals.get_step_distance()  # 例如，每步0.1米
# distance_threshold = globals.get_distance_threshold()  # 例如，1米的阈值
# STEPS = globals.get_steps #丝杠上下移动的步数

# 履带车型机器人，继承自AutoPaintingRobot
class TrackVehicle(AutoPaintingRobot):
    def __init__(self):
        super().__init__()
        #判断车是前进还是后退
        self.moving_forward_or_backward = True  # True 表示前进，False 表示后退
        # 初始化串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB', 9600, timeout=1)
        # 距离阈值
        self.distance_threshold = globals.distance_threshold

        self.imu_kalman_filter = KalmanFilter(
            F=np.array([[1]]),  # 状态转移矩阵F
            H=np.array([[1]]),  # 观测矩阵H
            Q=np.array([[0.1]]),  # 过程噪声协方差矩阵Q
            R=np.array([[0.1]])  # 观测噪声协方差矩阵R
        )

    def imu_callback(self, data):

        # 更新激光雷达数据
        self.imu_data = data
        # 更新当前朝向
        self.current_orientation = self.get_orientation_from_imu(data)
        
    def get_orientation_from_imu(self, data):
        # 检查IMU数据的有效性
        # IMU传感器提供的方向数据可能会无效或者不可用。
        # orientation_covariance是一个表示方向估计准确性的矩阵。
        # 如果这个矩阵的第一个元素小于0，它通常表示方向数据不可靠或未初始化。
        if data.orientation_covariance[0] < 0:
            return  # 如果数据无效，则提前返回，不进行进一步处理

        # 从IMU数据中获取四元数
        # 四元数是一种表示三维方向和旋转的数学方法，它可以避免万向锁问题。
        quaternion = geometry_msgs.msg.Quaternion()
        quaternion.x = data.orientation.x  # 获取四元数的X分量
        quaternion.y = data.orientation.y  # 获取四元数的Y分量
        quaternion.z = data.orientation.z  # 获取四元数的Z分量
        quaternion.w = data.orientation.w  # 获取四元数的W分量（实部）

        # 创建一个转换到目标坐标系的转换对象
        # 这里我们只关心旋转部分，所以仅设置四元数。
        transform = geometry_msgs.msg.TransformStamped()
        transform.transform.rotation = quaternion

        # 使用tf2转换器将四元数转换为欧拉角
        # 欧拉角是另一种表示三维空间中物体方向的方式。
        # 它包括滚转角（roll）、俯仰角（pitch）和偏航角（yaw）。
        current_orientation = tf2_geometry_msgs.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w])

        # 返回转换后的欧拉角
        # 函数返回一个包含三个角度（roll, pitch, yaw）的元组。
        return current_orientation

   
    def calculate_turn_angle(self, tree_position, current_orientation):
        # 假设current_orientation为机器人朝向的欧拉角
        # 计算机器人当前朝向与树木之间的角度差
        angle_to_tree = math.atan2(tree_position.y, tree_position.x)
        turn_angle = angle_to_tree - current_orientation.yaw
        return turn_angle 
 

    def convert_to_spray_claw_frame(self, tree_position):
        try:
            # 等待从激光雷达坐标系（/lidar_frame）到喷爪坐标系（/spray_claw_frame）的TF变换关系变得可用。
            # 这里设置最多等待4秒钟，rospy.Duration(4.0)表示等待时间。
            transform = self.tf_buffer.lookup_transform("/spray_claw_frame", "/lidar_frame", rospy.Time(0), rospy.Duration(4.0))

            # 进行坐标转换：将树木的位置从激光雷达坐标系转换到喷爪坐标系。
            tree_position_spray_claw_frame = tf2_geometry_msgs.do_transform_point(tree_position, transform)

            # 返回转换后的树木位置。
            return tree_position_spray_claw_frame

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # 如果在坐标转换过程中发生异常，比如找不到坐标系（LookupException）、
            # 网络连接问题（ConnectivityException）或者时间外推错误（ExtrapolationException），
            # 则记录错误信息并返回None。
            rospy.logerr("TF2转换异常: %s", e)
            return None

        
    def calculate_movement_for_spray(self, tree_position_spray_claw_frame):
        # 假设计算得到的距离是机器人与树木之间的直线距离
        movement_distance = math.sqrt(tree_position_spray_claw_frame.x ** 2 + tree_position_spray_claw_frame.y ** 2+tree_position_spray_claw_frame.z ** 2)
        return movement_distance
    


    # 创建控制履带车的串口指令
    def create_serial_command(self, RobotV, YawRate, StopFlag):
        # 设置指令的起始字节
        header = 0xAA

        # 设置指令的结束字节
        ender = 0x0D

        # 根据机器人的速度和偏航率计算右轮的速度
        rightVel = int(RobotV + YawRate * 0.05)  # 根据实际情况调整这个公式

        # 根据机器人的速度和偏航率计算左轮的速度
        leftVel = int(RobotV - YawRate * 0.05)   # 根据实际情况调整这个公式

        # 将右轮速度分解为高位和低位
        rightVelHigh, rightVelLow = (rightVel >> 8) & 0xFF, rightVel & 0xFF

        # 将左轮速度分解为高位和低位
        leftVelHigh, leftVelLow = (leftVel >> 8) & 0xFF, leftVel & 0xFF

        # 计算校验和，用于验证指令的完整性
        checksum = (header + StopFlag + rightVelHigh + rightVelLow + 
                    leftVelHigh + leftVelLow) & 0xFF
        
        # 将指令各部分组装成字节序列并返回
        return bytes([header, StopFlag, rightVelHigh, rightVelLow, 
                    leftVelHigh, leftVelLow, checksum, ender])


    # 发送控制履带车的串口指令
    def send_serial_command(self, command):
        try:
            self.serial_port.write(command)
        except serial.SerialException as e:
            rospy.logerr("Track vehicle serial communication error: %s", e)
    
    def send_movement_command(self, RobotV, YawRate, StopFlag):
        
        # RobotV: 代表机器人的速度。这是一个数值，用于控制机器人前进或后退的速度。
        # YawRate: 代表机器人的偏航率。这是一个数值，用于控制机器人的转向。
        # StopFlag: 代表停止标志。这通常是一个布尔值或整数，用于指示机器人是否应该停止。

        # 调用 create_serial_command 方法来生成串口命令。
        # 这个方法接受机器人的速度、偏航率和停止标志作为输入，并返回相应的串口命令。
        move_command = self.create_serial_command(RobotV, YawRate, StopFlag)

        # 调用 send_serial_command 方法发送生成的命令。
        # move_command 是由 create_serial_command 方法生成的串口命令。
        # send_serial_command 负责实际将命令通过串口发送给机器人，以控制其行动。
        self.send_serial_command(move_command)

    

    def move_towards_target(self): 
        # 从激光雷达数据中检测树木的位置
        tree_position = self.detect_tree_from_lidar(self.lidar_data)

        # 将检测到的树木位置转换到模块坐标系（例如，喷爪模块的坐标系）
        tree_position_module_frame = self.convert_to_spray_claw_frame(tree_position, self.listener)

        # 检查转换后的坐标是否有效
        if tree_position_module_frame is not None:
            # 计算从当前位置到树木位置的移动距离
            movement_distance = self.calculate_movement_for_spray(tree_position_module_frame)

            # 根据移动方向和距离阈值判断是前进、后退还是停止
            if self.moving_forward and movement_distance >= self.distance_threshold:
                # 如果是前进方向且距离大于等于阈值，则继续前进
                RobotV = 100  # 设置前进速度
                StopFlag = 0  # 不停止
            elif not self.moving_forward and movement_distance <= self.distance_threshold:
                # 如果是后退方向且距离小于等于阈值，则继续后退
                RobotV = -100  # 设置后退速度
                StopFlag = 0   # 不停止
            else:
                # 如果不满足上述条件，则停止移动
                RobotV = 0     # 停止移动
                StopFlag = 1   # 设置停止标志

            YawRate = 0  # 假设无需转向

            # 创建控制直线模组的串口指令
            # 发送控制指令
            self.send_serial_command(RobotV, YawRate, StopFlag)

    # 实现具体的喷涂树木逻辑
    def spray_tree(self):
        serial_data = self.read_serial_data()

        if serial_data == "OK":
            if AutoPaintingRobot.state_machine.state == 1:
                """
                使用三维雷达获取树的坐标及距离，IMU获取履带车的朝向计算出车要转动的角度，
                转换成控制指令，通过串口发送给下位机，调整车的朝向，并且需要考虑履带车和Y轴直线模组的长度，
                保证其在履带车转动过程中不会撞到树。

                调整车的朝向，正对树
                """
                # 获取树的位置
                tree_position = self.detect_tree_from_lidar(self.lidar_data)  # 假设lidar_data是已获取的数据

                # 获取当前朝向
                current_orientation = self.get_orientation_from_imu(self.imu_data)  # 假设imu_data是已获取的数据

                # 计算转动角度
                turn_angle = self.calculate_turn_angle(tree_position, current_orientation)
                #turn_angle = 0
                # 确定机器人的速度和停止标志
                RobotV = 100  # 假设的速度值
                StopFlag = 0  # 假设的停止标志，0 表示不停止

                # 计算YawRate（偏航率），假设与转动角度相关
                YawRate = turn_angle * 0.1  # 这里的0.1是一个假设的比例因子，需要根据实际情况调整

                # 创建控制指令
                # 发送控制指令
                self.send_serial_command(RobotV, YawRate, StopFlag)

                #更新状态量
                AutoPaintingRobot.state_machine.update_state(2)

            elif AutoPaintingRobot.state_machine.state == 3:
                """
                使用三维雷达获取树的坐标及距离，通过TF坐标转换成相对于喷爪末端的坐标和距离，
                计算出车要移动的距离，转换成控制指令，通过串口发送给下位机，喷爪闭合。

                使车前进合适距离
                """
                # 例如，根据某些条件决定是前进还是后退
                self.moving_forward_or_backward = True  # 或 False，根据需要设置

                # 调用移动函数
                self.move_towards_target(self.distance_threshold)
                #更新状态量
                AutoPaintingRobot.state_machine.update_state(4)

            elif AutoPaintingRobot.state_machine.state == 5:
                """
                使车后退合适距离
                """
                # 例如，根据某些条件决定是前进还是后退
                self.moving_forward_or_backward = False  # 或 False，根据需要设置

                # 调用移动函数
                self.move_towards_target(self.distance_threshold)
                #更新状态量
                AutoPaintingRobot.state_machine.update_state(6)

        elif serial_data == "ERROR":
            # 错误处理逻辑
            rospy.logwarn("Serial communication error in TrackVehicle during spraying")
            # 可以在此处添加更多的错误处理逻辑，例如：
            # - 将机器人移动到安全位置
            # - 发送错误通知到控制中心
            # - 尝试重新建立串口连接