#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# 初始化速度变量
linear_speed = 0.0
angular_speed = 0.0

multiple_linear_speed = 0.5
multiple_angular_speed = 0.5

def callback(data):
    global linear_speed, angular_speed,multiple_linear_speed,multiple_angular_speed
    twist = Twist()

    
    # 使用左摇杆的垂直方向控制前进后退的基础速度
    base_linear_speed = data.axes[1] * multiple_linear_speed
    
    # 使用右摇杆的水平方向控制左右转向的基础速度
    base_angular_speed = data.axes[3] * multiple_angular_speed

    #  左肩键————>线速度倍值加减
    if data.buttons[4] == 1:  # L1 增加线速度
        multiple_linear_speed += 0.1
        multiple_linear_speed = max(0.2, multiple_linear_speed)  # 最小值为0.2
        print("**************max linear speed   ++")
    if data.axes[2] == -1.0:  # L2 减少线速度
        multiple_linear_speed -= 0.1
        multiple_linear_speed = max(0.2, multiple_linear_speed)  # 最小值为0.2
        print("**************max linear speed   --")
        

    # 右肩键————>角速度倍值加减
    if data.buttons[5] == 1:  # R1 增加角速度
        multiple_angular_speed += 0.1
        multiple_angular_speed = max(0.2, multiple_angular_speed)  # 最小值为0.2
        print("**************max angular speed   ++")
    if data.axes[5] == -1.0:  # R2 减少角速度
        multiple_angular_speed -= 0.1
        multiple_angular_speed = max(0.2, multiple_angular_speed)  # 最小值为0.2
        print("**************max angular speed   --")

    #slect键和start键速度重置为0
    if (data.buttons[8] == 1 and data.buttons[9] == 1 ):
        twist.linear.x = 0
        twist.angular.z = 0
    else:
        twist.linear.x = data.axes[1] * multiple_linear_speed 
        twist.angular.z = data.axes[3] * multiple_angular_speed

    # twist.linear.x = data.axes[1] * multiple_linear_speed 
    # twist.angular.z = data.axes[3] * multiple_angular_speed

    pub.publish(twist)

def start():
    rospy.init_node('joy_to_twist')
    global pub
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    print("_____joy_to_twist is ready, Are you ok?       <(￣︶￣)↗[GO!]")
    
    rospy.spin()

if __name__ == '__main__':
    start()
