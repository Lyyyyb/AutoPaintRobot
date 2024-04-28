#include <ros/ros.h> // 引入ROS的核心头文件
#include <sensor_msgs/Joy.h> // 引入用于处理游戏手柄输入的消息类型
#include <geometry_msgs/Twist.h> // 引入用于表示速度的消息类型
#include <boost/bind.hpp> // 引入Boost库中的bind功能
#include <functional> // 引入C++标准库的功能支持

// 常量定义
const double MAX_LINEAR_SPEED = 1.0; // 定义最大线速度，单位为米/秒
const double MIN_SPEED_THRESHOLD = 0.1; // 定义最小速度阈值，用于避免微小的输入导致的机器人运动
const double WHEEL_DISTANCE = 0.5; // 定义轮间距离，单位为米

ros::Publisher cmd_vel_pub; // 定义一个发布者对象，用于发布cmd_vel话题，控制机器人速度

// 手柄消息回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // 打印手柄线速度和角速度信息，用于调试
    ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]);

    double linear_vel = msg->axes[1]; // 从手柄输入获取线速度
    double angular_vel = msg->axes[3]; // 从手柄输入获取角速度
    geometry_msgs::Twist cmd_vel; // 创建Twist消息对象，用于存储机器人的速度命令

    // 根据手柄输入调整机器人的运动命令
    if (fabs(linear_vel) > MIN_SPEED_THRESHOLD) { // 如果线速度大于最小阈值
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 根据手柄的线性输入调整实际的线速度
        cmd_vel.angular.z = angular_vel; // 直接设置角速度
    } else if (fabs(angular_vel) > MIN_SPEED_THRESHOLD) { // 如果角速度大于最小阈值
        cmd_vel.linear.x = 0; // 将线速度设置为0，停止直线运动
        cmd_vel.angular.z = angular_vel * WHEEL_DISTANCE; // 调整角速度，考虑到轮间距离
    } else { // 如果手柄输入低于阈值
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 保持线速度
        cmd_vel.angular.z = angular_vel; // 保持角速度
    }

    cmd_vel_pub.publish(cmd_vel); // 发布调整后的运动命令到cmd_vel话题，控制机器人运动
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
