#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <functional>

// 常量定义
double MAX_LINEAR_SPEED = 1.0;
double MIN_SPEED_THRESHOLD = 0.1;
double WHEEL_DISTANCE = 0.5;

ros::Publisher cmd_vel_pub;

// 手柄消息回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]);

    double linear_vel = msg->axes[1];
    double angular_vel = msg->axes[3];
    geometry_msgs::Twist cmd_vel;

    // 避免重复计算fabs(angular_vel)
    double abs_angular_vel = fabs(angular_vel);

    if (fabs(linear_vel) > MIN_SPEED_THRESHOLD) {
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED;
        cmd_vel.angular.z = angular_vel;
    } else if (abs_angular_vel > MIN_SPEED_THRESHOLD) {
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = angular_vel * WHEEL_DISTANCE;
    } else {
        cmd_vel.linear.x = linear_vel * MAX_LINEAR_SPEED;
        cmd_vel.angular.z = angular_vel;
    }

    cmd_vel_pub.publish(cmd_vel);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_to_cmd_vel_node");
    ros::NodeHandle nh;

    // 添加错误检查
    if (!nh.hasParam("max_linear_speed") || !nh.hasParam("min_speed_threshold") || !nh.hasParam("wheel_distance")) {
        ROS_ERROR("Failed to get parameters");
        return -1;
    }

    nh.getParam("max_linear_speed", MAX_LINEAR_SPEED);
    nh.getParam("min_speed_threshold", MIN_SPEED_THRESHOLD);
    nh.getParam("wheel_distance", WHEEL_DISTANCE);

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCallback);

    ros::spin();

    return 0;
}
