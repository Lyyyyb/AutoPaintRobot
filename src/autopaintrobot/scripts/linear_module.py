import rospy
import serial
from auto_painting_robot import AutoPaintingRobot
import globals
import tf2_geometry_msgs  # 用于坐标转换
import tf2_ros
from robot_state import RobotState,SprayingStateMachine
import time

# #步进电机一步滑台移动的距离
# STEP_DISTANCE = globals.get_step_distance()  # 例如，每步0.1米
# distance_threshold = globals.get_distance_threshold()  # 例如，1米的阈值
# STEPS = globals.get_steps #丝杠上下移动的步数

# 直线模组，继承自AutoPaintingRobot
class LinearModule(AutoPaintingRobot):
    def __init__(self,step_distance, step_y, speed, mode):
        super().__init__()

        # 初始化串口通信
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)

        # 使用构造函数参数初始化成员变量
        self.step_distance = step_distance  # 将步距（每步移动的距离）保存为成员变量
        self.STEP_Y = step_y  # 将Y轴步长（Y轴每步移动的距离）保存为成员变量
        self.speed = speed  # 将速度配置（移动速度）保存为成员变量
        self.mode = mode  # 将模式设置（可能影响机器人的行为或配置）保存为成员变量
        self.STEP_X = 0 # X轴需要动的步数初始化

    # 创建控制直线模组的串口指令
    def create_serial_command(self,x, y, speed, mode):
        # 格式化命令字符串
        # x: 通常代表X坐标或位置
        # y: 通常代表Y坐标或位置
        # speed: 代表移动速度
        # mode: 代表操作模式，可能是指定不同的行为或配置
        command = f"move {x} {y} {speed} {mode}"
        
        # 返回格式化后的命令
        return command


    # 发送控制直线模组的串口指令
    def send_serial_command(self, command):
        # command: 这是要发送的命令字符串。

        try:
            # 尝试执行的代码块。
            # 使用串口对象的 write 方法发送命令。
            # command.encode() 将字符串转换为字节，因为串行通信通常需要字节数据。
            self.serial_port.write(command.encode())

        except serial.SerialException as e:
            # 如果在尝试执行 try 代码块时发生了 serial.SerialException 异常，

            # rospy.logerr 用于记录错误信息。
            # 这里记录了一个错误消息，说明了串口通信出现了问题，并打印了异常详情。
            # 这对于调试和日志记录非常有用。
            rospy.logerr("Linear module serial communication error: %s", e)



    def convert_to_module_frame(self, tree_position):
        try:
            # 使用tf2的lookup_transform方法获取从lidar_frame到module_frame的转换关系。
            # "/module_frame" 是目标坐标系，"/lidar_frame" 是源坐标系。
            # rospy.Time(0) 表示获取最近的可用转换，rospy.Duration(4.0) 设置了4秒的等待时间。
            trans = self.tf_buffer.lookup_transform("/module_frame", "/lidar_frame", rospy.Time(0), rospy.Duration(4.0))

            # 使用tf2_geometry_msgs的do_transform_point方法，根据上一步获取的转换关系，将点（tree_position）从lidar_frame转换到module_frame。
            # 这里的tree_position是一个PointStamped类型，表示在lidar_frame坐标系下的一个点。
            tree_position_module_frame = tf2_geometry_msgs.do_transform_point(tree_position, trans)

            # 返回转换后的点的坐标，这里只返回点的位置部分（不包含时间戳等其他信息）。
            return tree_position_module_frame.point

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # 如果在转换过程中出现任何异常（如找不到转换关系、坐标系之间连接问题或时间外推问题），则记录错误日志。
            rospy.logerr("TF2转换异常: %s", e)

            # 发生异常时，返回None。
            return None
        
    def calculate_movement_for_module(self, tree_position_module_frame):
        # 假设直线模组的起点为(0,0,0)，计算滑台需要移动的距离
        movement_distance = tree_position_module_frame.x  # 根据实际情况可能需要调整
        return movement_distance
    
    def send_movement_command(self, x_steps, y_steps):
        """
        发送移动指令到串口。
        :param x_steps: X轴需要移动的步数。
        :param y_steps: Y轴需要移动的步数。
        """
        move_command = self.create_serial_command(x_steps, y_steps, self.speed, self.mode)
        self.send_serial_command(move_command)
    
    def move_and_operate_spray_claw(self):
        # 检测树木的位置，使用来自激光雷达的数据
        tree_position = self.detect_tree_from_lidar(self.lidar_data)

        # 将检测到的树木位置从激光雷达坐标系转换到模块坐标系
        tree_position_module_frame = self.convert_to_module_frame(tree_position, self.listener)

        # 检查转换结果是否为None
        if tree_position_module_frame is None:
            rospy.logerr("坐标转换失败，无法进行移动操作")
            return  # 结束函数执行

        # 计算从当前位置到目标位置的移动距离
        movement_distance = self.calculate_movement_for_module(tree_position_module_frame)

        # 根据移动距离计算出步进电机需要走的步数
        self.STEP_X = self.calculate_steps_for_motor(movement_distance, self.step_distance)

        # 创建并发送串口指令以控制机器人移动相应的步数
        # 发送移动指令
        self.send_movement_command(self.STEP_X, 0)

        # 控制喷爪的操作
        #self.open_spray_claw()


    
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

    def set_pump_duty_cycle(self, duty_cycle):
        """
        发送指令以设置水泵的占空比。

        :param duty_cycle: 要设置的占空比，范围从0到100。
        """
        if 0 <= duty_cycle <= 100:
            # 构造指令字符串，格式为 "pump <duty_cycle>"
            command = f"pump {duty_cycle}"
            self.send_serial_command(command)
        else:
            rospy.logerr("Invalid pump duty cycle: %d. Must be between 0 and 100.", duty_cycle)
    

    def wait_for_ok(self, timeout=10):
        """
        循环读取串口数据，直到接收到"OK"或达到超时时间。
        :param timeout: 超时时间（秒）
        """
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                rospy.logwarn("Timeout while waiting for OK.")
                break  # 超时退出循环

            serial_data = self.read_serial_data()
            if serial_data == "OK":
                break  # 如果读取到"OK"，退出循环
            elif serial_data == "ERROR":
                rospy.logwarn("Error while waiting for OK. Received: %s", serial_data)
                # 可以根据需求添加额外的错误处理逻辑
                # 比如重新尝试发送命令，或者中断操作等




    # 实现具体的喷涂树木逻辑
    def spray_tree(self):
        serial_data = self.read_serial_data()

        if serial_data == "OK":
            if AutoPaintingRobot.state_machine.state == 2:
                """
                调整车的朝向后，使用三维雷达获取树的坐标，通过TF坐标转换成相对于X轴直线模组最左侧的坐标，
                通过坐标信息计算出X轴直线模组的滑台需要移动多少距离，进而得到X轴直线模组的步进电机需要运动的步数，
                转换成控制指令，通过串口发送给下位机。

                调整X轴丝杠，正对树，张开喷爪
                """
                #控制丝杠和喷爪
                self.move_and_operate_spray_claw(self.step_distance)
                #更新状态量
                AutoPaintingRobot.state_machine.update_state(3)

            elif AutoPaintingRobot.state_machine.state == 4:
                """
                控制水泵喷漆，控制Z轴直线模组的滑台，使Y轴直线模组上下移动对树均匀喷漆。
                控制Y轴直线模组末端的喷爪张开

                喷爪闭合，控制Z轴丝杠，使Y轴丝杠上下运动，同时控制水泵喷水，喷涂结束，喷爪张开
                """
                # 发送指令以闭合喷涂装置的喷爪
                #self.close_spray_claw()
                # 创建并发送串口指令以控制机器人移动相应的步数
                # 假设：我们将 X 和 Y 设为步数，速度和模式为预设值
                # 设置水泵的占空比为50%。这通常用于启动水泵或设置其运行速度。
                # 在PWM控制中，占空比决定了输出功率的多少。这里，50%的占空比可能意味着水泵运行在中等速度。
                self.set_pump_duty_cycle(50)

                # 发送一个控制命令，让机器沿着Y轴正方向移动特定的步数。
                # self.STEP_Y是预定义的步数，表示机器应该移动的距离。
                # 第一个参数0表示X轴的步数，这里设为0，说明X轴不移动。
                self.send_movement_command(0, self.STEP_Y)

                # 阻塞等待直到从机器接收到"OK"信号。
                # 这通常是一个同步机制，确保机器已经完成上一个命令之后才继续执行后续操作。
                self.wait_for_ok()

                # 发送另一个控制命令，让机器沿着Y轴反方向移动相同的步数。
                # 这通常用于将机器返回到起始位置或移至新的预定位置。
                # 通过使用-self.STEP_Y，我们指定机器在Y轴上反向移动。
                self.send_movement_command(0, -self.STEP_Y)

                # 再次阻塞等待，直到从机器接收到"OK"信号。
                # 这确保了机器已经完成反向移动命令。
                self.wait_for_ok()

                # 将水泵的占空比设置为0%，通常意味着停止水泵的运行。
                # 在一些应用中，这可能用于停止流体的流动或确保机器在下一个步骤之前不再进行任何动作。
                self.set_pump_duty_cycle(0)

                #更新状态量
                AutoPaintingRobot.state_machine.update_state(5)

            elif AutoPaintingRobot.state_machine.state == 6:
                """
                喷爪闭合
                """
                #闭合喷爪
                #self.close_spray_claw()
                #更新状态量
                AutoPaintingRobot.state_machine.update_state(7)
                #更新状态量
                AutoPaintingRobot.state = RobotState.NAVIGATING

        elif serial_data == "ERROR":
            # 错误处理逻辑
            rospy.logwarn("Serial communication error in LinearModule during spraying")
            # 可以在此处添加更多的错误处理逻辑