import rospy
import can
import math
from sensor_msgs.msg import Joy
import sys
sys.path.append('/home/lyb/AutoPaintRobot/src/can_test/scripts')
from pcan_cybergear import CANMotorController



class JoyToMotorController:
    def __init__(self):
        rospy.init_node('joy_to_pcan_node')
        # 创建PCAN适配器对象
        self.bus = can.interface.Bus(bustype='pcan', channel='PCAN_USBBUS1', bitrate=500000)
        # self.bus = can.interface.Bus(bustype='socketcan', channel='vcan0', bitrate=500000)

        # 使用PCAN适配器对象初始化CANMotorController
        self.controller = CANMotorController(self.bus)
        self.sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.button_3_pressed = False
        self.button_0_pressed = False

    def joy_callback(self, data):
        if data.buttons[3] == 1 and not self.button_3_pressed:
            joy_to_motor_controller.controller.enable()
            self.controller.send_motor_control_command(torque=1.0, target_angle=3.14, target_velocity=30.0, Kp=100.0, Kd=1.0)
            self.button_3_pressed = True
        elif data.buttons[3] == 0 and self.button_3_pressed:
            self.controller.send_motor_control_command(torque=0.0, target_angle=0.0, target_velocity=0.0, Kp=0.0, Kd=0.0)
            joy_to_motor_controller.controller.disable()
            self.button_3_pressed = False

        if data.buttons[0] == 1 and not self.button_0_pressed:
            joy_to_motor_controller.controller.enable()
            self.controller.send_motor_control_command(torque=-1.0, target_angle=-3.14, target_velocity=-30.0, Kp=100.0, Kd=1.0)
            self.button_0_pressed = True
        elif data.buttons[0] == 0 and self.button_0_pressed:
            self.controller.send_motor_control_command(torque=0.0, target_angle=0.0, target_velocity=0.0, Kp=0.0, Kd=0.0)
            joy_to_motor_controller.controller.disable()
            self.button_0_pressed = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    joy_to_motor_controller = JoyToMotorController()

    joy_to_motor_controller.run()


    # joy_to_motor_controller.controller.send_motor_control_command(torque=1.0, target_angle=180.0, target_velocity=5.0, Kp=100.0, Kd=1.0)


    joy_to_motor_controller.controller.send_motor_control_command(torque=1.0, target_angle=math.pi, target_velocity=0.1, Kp=100.0, Kd=1.0)
