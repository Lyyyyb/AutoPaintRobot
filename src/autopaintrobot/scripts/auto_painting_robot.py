import rospy
from sensor_msgs.msg import LaserScan
from robot_state import RobotState,SprayingStateMachine
from kalman_filter import KalmanFilter
import numpy as np
import serial
import tf2_ros
import geometry_msgs.msg

# 自动喷漆机器人的父类
class AutoPaintingRobot:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('auto_painting_robot')
        # 订阅激光雷达数据
        self.lidar_sub = rospy.Subscriber('/lidar', LaserScan, self.lidar_callback)
        # 设置初始状态为导航
        self.state = RobotState.NAVIGATING
        # 初始化状态机
        self.state_machine = SprayingStateMachine()

        # tf2_ros.TransformBroadcaster用于发布坐标系之间的变换关系
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # tf2_ros.Buffer用于存储坐标系之间的变换关系
        self.tf_buffer = tf2_ros.Buffer()
        # tf2_ros.TransformListener用于监听和接收坐标系之间的变换关系
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        #初始化雷达数据
        self.lidar_data = None
        #初始化串口
        self.serial_port = None
        # 直接在代码中设置TF变换的参数   
        # 设置TF变换的参数，分别代表模块框架和喷爪框架的位置和旋转
        self.module_transform = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # [tx, ty, tz, rx, ry, rz, rw]
        self.claw_transform = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # 平移和旋转的参数（平移x, y, z，旋转四元数x, y, z, w）
        self.lidar_kalman_filter = KalmanFilter(
            F=np.array([[1, 0], [0, 1]]),  # 状态转移矩阵F
            H=np.array([[1, 0]]),          # 观测矩阵H
            Q=np.array([[0.1, 0], [0, 0.1]]),  # 过程噪声协方差矩阵Q
            R=np.array([[0.1]])            # 观测噪声协方差矩阵R
        )

    # 激光雷达数据的回调函数
    def lidar_callback(self, data):
        # 更新激光雷达数据
        self.lidar_data = data
        # 处理激光雷达数据以便找到树木的位置
        # self.target_tree_position = self.detect_tree_from_lidar(data)
    

    # 解析激光雷达数据以找到树木，
    def detect_tree_from_lidar(self, data):
        # 假设 data 提供了直接的 X, Y, Z 坐标信息
        # 这里是一个简化的示例，具体实现应根据实际数据格式进行
        x = data.x  # 从 data 中获取 X 坐标
        y = data.y  # 从 data 中获取 Y 坐标
        z = data.z  # 从 data 中获取 Z 坐标

        # 将坐标赋值给 position
        position = (x, y, z)
        return position

    # 生成串口指令，需在子类中具体实现
    def create_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")

    # 发送串口指令，需在子类中具体实现
    def send_serial_command(self, command):
        raise NotImplementedError("Must be implemented in subclass")
    
    # 读取串口数据
    def read_serial_data(self):
        # 尝试读取串口数据，并处理可能的异常
        try:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode().strip()
                if data:
                    return data
        except serial.SerialException as e:
            rospy.logerr("Error reading serial data: %s", e)
            # 可以在此处添加更多的错误处理逻辑，例如：
            # - 尝试重新初始化串口连接
            # - 将错误状态发送到一个监控系统
            # - 如果错误频繁发生，可以采取降级策略，比如减少读取频率等
        return "ERROR"
    
        # 发布TF变换
    def publish_transforms(self):
        
        current_time = rospy.Time.now()

        # 使用成员变量发布TF变换
        # 使用成员变量发布TF变换，*号表示从列表解包参数
        self.send_transform(*self.module_transform, current_time, "/lidar_frame", "/module_frame")
        self.send_transform(*self.claw_transform, current_time, "/lidar_frame", "/spray_claw_frame")

    # 发送单个TF变换
    def send_transform(self, tx, ty, tz, rx, ry, rz, rw, time, parent_frame, child_frame):

        t = geometry_msgs.msg.TransformStamped()  # 创建一个 TransformStamped 消息对象
        t.header.stamp = time                     # 设置消息的时间戳
        t.header.frame_id = parent_frame          # 设置父坐标系的名称
        t.child_frame_id = child_frame            # 设置子坐标系的名称

        # 设置坐标变换中的平移部分
        t.transform.translation.x = tx            # 设置x轴上的平移
        t.transform.translation.y = ty            # 设置y轴上的平移
        t.transform.translation.z = tz            # 设置z轴上的平移

        # 设置坐标变换中的旋转部分（使用四元数）
        t.transform.rotation.x = rx               # 四元数的x分量
        t.transform.rotation.y = ry               # 四元数的y分量
        t.transform.rotation.z = rz               # 四元数的z分量
        t.transform.rotation.w = rw               # 四元数的w分量（实部）

        self.tf_broadcaster.sendTransform(t)      # 使用tf2的广播器发送变换


    
    # 导航到树木的逻辑
    def navigate_to_tree(self):
        # 读取串口数据并根据数据内容更新状态
        serial_data = self.read_serial_data()  
        if serial_data == "OK" and self.state_machine.state == 7:
            # 正常逻辑
            self.state_machine.update_state(1)
            self.state = RobotState.SPRAYING
        elif serial_data == "ERROR":
            # 错误处理
            rospy.logwarn("Serial communication error during navigation")
            # 根据具体情况执行错误恢复逻辑，例如：
            # - 将机器人移动到安全位置
            # - 发送错误通知到控制中心
            # - 尝试重新建立串口连接

    # 喷涂树木的逻辑，需在子类中具体实现
    def spray_tree(self):
        # 喷树逻辑
        # ...
        # 假设喷树完成，切换回导航状态
        self.state = RobotState.NAVIGATING
