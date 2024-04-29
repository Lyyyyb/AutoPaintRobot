#include <ros/ros.h> // 引入ROS相关的头文件
#include <sensor_msgs/Joy.h> // 引入手柄消息的头文件
#include <geometry_msgs/Twist.h> // 引入用于控制机器人运动的消息类型头文件
#include <boost/bind.hpp> // 引入Boost库中的bind功能头文件
#include <functional> // 引入C++标准库中功能相关的头文件

// 常量定义
double MAX_LINEAR_SPEED = 1.0; // 定义最大线速度
double MIN_SPEED_THRESHOLD = 0.1; // 定义速度的最小阈值，低于此速度不进行处理
double WHEEL_DISTANCE = 0.5; // 轮距，用于计算转弯时的角速度

ros::Publisher cmd_vel_pub; // 定义一个发布者，用来发送速度控制指令

// 手柄控制消息的回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]); // 打印线速度和角速度信息

    double linear_vel = msg->axes[1]; // 获取手柄的线速度控制轴的值
    double angular_vel = msg->axes[3]; // 获取手柄的角速度控制轴的值
    geometry_msgs::Twist cmd_vel; // 创建Twist消息，用于发送速度指令

    // 避免重复调用fabs函数，优化性能
    double abs_angular_vel = fabs(angular_vel);

    // 根据速度的阈值判断如何发布速度指令
    if (fabs(linear_vel) > MIN_SPEED_THRESHOLD) {
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 线速度赋值，考虑最大速度限制
        cmd_vel.angular.z = angular_vel; // 角速度直接赋值
    } else if (abs_angular_vel > MIN_SPEED_THRESHOLD) {
        cmd_vel.linear.x = 0; // 线速度设置为0
        cmd_vel.angular.z = angular_vel * WHEEL_DISTANCE; // 角速度乘以轮距得到更合适的转弯速度
    } else {
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED; // 低速下直接控制线速度
        cmd_vel.angular.z = angular_vel; // 低速下直接控制角速度
    }

    cmd_vel_pub.publish(cmd_vel); // 发布速度控制指令
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_to_cmd_vel_node"); // 初始化ROS节点
    ros::NodeHandle nh; // 创建节点句柄
    // 获取参数值
    nh.getParam("max_linear_speed", MAX_LINEAR_SPEED);
    nh.getParam("min_speed_threshold", MIN_SPEED_THRESHOLD);
    nh.getParam("wheel_distance", WHEEL_DISTANCE);


    // 参数检查，确保必要的参数已经设置
    if (!nh.hasParam("max_linear_speed") || !nh.hasParam("min_speed_threshold") || !nh.hasParam("wheel_distance")) {
        ROS_ERROR("Failed to get parameters"); // 参数获取失败的错误信息
        return -1; // 返回-1，表示程序异常退出
    }



    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); // 创建一个发布者，用于发布速度控制指令

    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallback); // 订阅手柄控制的消息，设置回调函数

    ros::spin(); // 进入ROS消息处理循环，等待回调函数的触发

    return 0; // 程序正常退出
}
