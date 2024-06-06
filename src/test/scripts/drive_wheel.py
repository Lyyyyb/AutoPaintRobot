#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class WheelTeleop:
    def __init__(self):
        rospy.init_node('wheel_teleop')
        self.left_pub = rospy.Publisher('/LF_wheel_controller/command', Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/RF_wheel_controller/command', Float64, queue_size=1)
        self.left_back_pub = rospy.Publisher('/LB_wheel_controller/command', Float64, queue_size=1)
        self.right_back_pub = rospy.Publisher('/RB_wheel_controller/command', Float64, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self.callback)

    def callback(self, msg):
        # 假设直接将线速度 v 分配给所有车轮，角速度 w 分配影响左右轮速差
        v = msg.linear.x
        w = msg.angular.z
        
        # 这里反转方向
        left_speed = -(v - w)
        right_speed = -(v + w)

        # 发布到各自的控制器
        self.left_pub.publish(Float64(left_speed))
        self.right_pub.publish(Float64(right_speed))
        self.left_back_pub.publish(Float64(left_speed))
        self.right_back_pub.publish(Float64(right_speed))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = WheelTeleop()
        node.run()
    except rospy.ROSInterruptException:
        pass
