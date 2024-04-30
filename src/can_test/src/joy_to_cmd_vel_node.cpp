#include <ros/ros.h> // 引入ROS相关的头文件
#include <sensor_msgs/Joy.h> // 引入手柄消息的头文件
#include <geometry_msgs/Twist.h> // 引入用于控制机器人运动的消息类型头文件
#include <boost/bind.hpp> // 引入Boost库中的bind功能头文件
#include <functional> // 引入C++标准库中功能相关的头文件

// 常量定义
double MAX_LINEAR_SPEED = 1.0; // 定义最大线速度
double MIN_SPEED_THRESHOLD = 0.1; // 定义速度的最小阈值，低于此速度不进行处理
double WHEEL_DISTANCE = 0.8; // 轮距，用于计算转弯时的角速度

double scale_factor = 0.5; // 可以根据需要调整这个值
ros::Publisher cmd_vel_pub; // 定义一个发布者，用来发送速度控制指令

// 手柄控制消息的回调函数
// void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
//     ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]); // 打印线速度和角速度信息

//     double linear_vel = msg->axes[1] * scale_factor; // 应用缩放因子
//     double angular_vel = msg->axes[3] * scale_factor; // 应用缩放因子
//     geometry_msgs::Twist cmd_vel; // 创建Twist消息，用于发送速度指令

//     // 避免重复调用fabs函数，优化性能
//     double abs_angular_vel = fabs(angular_vel);

//     // 根据速度的阈值判断如何发布速度指令
//     if (fabs(linear_vel) > MIN_SPEED_THRESHOLD) {
//         cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 线速度赋值，考虑最大速度限制
//         cmd_vel.angular.z = angular_vel; // 角速度直接赋值
//     } else if (abs_angular_vel > MIN_SPEED_THRESHOLD) {
//         cmd_vel.linear.x = 0; // 线速度设置为0
//         cmd_vel.angular.z = angular_vel * WHEEL_DISTANCE; // 角速度乘以轮距得到更合适的转弯速度
//     } else {
//         cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 低速下直接控制线速度
//         cmd_vel.angular.z = angular_vel; // 低速下直接控制角速度
//     }

//     cmd_vel_pub.publish(cmd_vel); // 发布速度控制指令
// }

// 手柄控制消息的回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    double scale_factor = std::pow(fabs(msg->axes[1]), 0.5); // 使用开方非线性调整线速度的缩放因子
    double linear_vel = msg->axes[1] * scale_factor * MAX_LINEAR_SPEED; // 应用缩放因子并乘以最大速度
    double angular_vel = msg->axes[3] * scale_factor * WHEEL_DISTANCE; // 应用缩放因子并考虑轮距影响，增强转向性能

    geometry_msgs::Twist cmd_vel; // 创建Twist消息，用于发送速度指令

    // 确保输出速度符合安全阈值
    if (fabs(linear_vel) > MIN_SPEED_THRESHOLD || fabs(angular_vel) > MIN_SPEED_THRESHOLD) {
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel / WHEEL_DISTANCE; // 角速度除以轮距，调整为适合的转速
    } else {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }

    cmd_vel_pub.publish(cmd_vel); // 发布速度控制指令
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_to_cmd_vel_node");
    ros::NodeHandle nh;

    nh.param("/joy_to_cmd_vel_node/max_linear_speed", MAX_LINEAR_SPEED, 1.0);
    nh.param("/joy_to_cmd_vel_node/min_speed_threshold", MIN_SPEED_THRESHOLD, 0.1);
    nh.param("/joy_to_cmd_vel_node/wheel_distance", WHEEL_DISTANCE, 0.8);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallback);

    ros::spin();
    return 0;
}