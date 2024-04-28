#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <functional>

// 常量定义
const double MAX_LINEAR_SPEED = 1.0; // 最大线速度
const double MIN_SPEED_THRESHOLD = 0.1; // 最小速度阈值
const double WHEEL_DISTANCE = 0.5; // 轮间距离

ros::Publisher cmd_vel_pub; // cmd_vel话题发布者

// 手柄消息回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // 打印手柄线速度和角速度信息
    ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]);

    double linear_vel = msg->axes[1]; // 获取线速度
    double angular_vel = msg->axes[3]; // 获取角速度
    geometry_msgs::Twist cmd_vel; // 创建Twist消息对象

    // 根据手柄输入调整机器人的运动命令
    if (fabs(linear_vel) > MIN_SPEED_THRESHOLD) { // 如果线速度大于最小阈值
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 设置线速度
        cmd_vel.angular.z = angular_vel; // 设置角速度
    } else if (fabs(angular_vel) > MIN_SPEED_THRESHOLD) { // 如果角速度大于最小阈值
        cmd_vel.linear.x = 0; // 设置线速度为零
        cmd_vel.angular.z = angular_vel * WHEEL_DISTANCE; // 设置角速度，以转换为轮速度
    } else { // 否则
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 设置线速度
        cmd_vel.angular.z = angular_vel; // 设置角速度
    }

    cmd_vel_pub.publish(cmd_vel); // 发布调整后的运动命令
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_to_cmd_vel_node");
    ros::NodeHandle nh;

    // Advertise the cmd_vel publisher
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Subscribe to the joy topic
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallback);
    ros::spin();
	return 0;
}
