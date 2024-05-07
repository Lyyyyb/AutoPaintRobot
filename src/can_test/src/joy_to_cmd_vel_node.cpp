#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <functional>
#include <cmath> // 引入数学库，用于执行数学运算

// 常量定义
double MAX_LINEAR_SPEED = 1.0; // 最大线速度，单位：米/秒
double MIN_SPEED_THRESHOLD = 0.1; // 最小速度阈值，低于此速度的指令将被忽略
double WHEEL_DISTANCE = 0.8; // 轮间距，用于计算转向时的角速度
double JOYSTICK_DEADZONE = 0.05; // 手柄的死区值，避免微小的操作误差导致的机器人移动
double MIN_ANGULAR_VELOCITY = 0.1; // 最小角速度，单位：弧度/秒
double scale_factor = 0.5; // 缩放因子，用于调整手柄输入的影响程度
ros::Publisher cmd_vel_pub; // 发布者，用来发送速度控制指令到cmd_vel话题
double linear_scale_exponent = 0.5; // 线速度的非线性调整因子，用于调整手柄输入到实际速度的映射关系
double angular_scale_exponent = 0.5; // 角速度的非线性调整因子
double speed_smoothing_factor = 0.2; // 速度平滑处理因子，用于平滑速度变化
geometry_msgs::Twist cmd_vel; // 用于存储和发布的速度指令

// 手柄控制消息的回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]); // 打印当前手柄的线速度和角速度输入

    // 根据手柄按键调整最大线速度和最小角速度
    if (msg->buttons[4] == 1) { // L1按键增加线速度
        MAX_LINEAR_SPEED += 0.1;
        ROS_INFO("Max linear speed increased to: %f", MAX_LINEAR_SPEED);
    }
    if (msg->axes[2] == -1.0) { // L2按键减少线速度
        MAX_LINEAR_SPEED -= 0.1;
        ROS_INFO("Max linear speed decreased to: %f", MAX_LINEAR_SPEED);
    }
    // 确保线速度不低于0.2米/秒
    MAX_LINEAR_SPEED = std::max(0.2, MAX_LINEAR_SPEED);

    if (msg->buttons[5] == 1) { // R1按键增加角速度
        MIN_ANGULAR_VELOCITY += 0.1;
        ROS_INFO("Min angular velocity increased to: %f", MIN_ANGULAR_VELOCITY);
    }
    if (msg->axes[5] == -1.0) { // R2按键减少角速度
        MIN_ANGULAR_VELOCITY -= 0.1;
        ROS_INFO("Min angular velocity decreased to: %f", MIN_ANGULAR_VELOCITY);
    }
    // 确保角速度不低于0.2弧度/秒
    MIN_ANGULAR_VELOCITY = std::max(0.2, MIN_ANGULAR_VELOCITY);

    // 计算最终的线速度和角速度
    double linear_vel = -msg->axes[1] * MAX_LINEAR_SPEED;
    double angular_vel = msg->axes[3] * MIN_ANGULAR_VELOCITY;

    // 设置速度指令
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_to_cmd_vel_node"); // 初始化节点
    ros::NodeHandle nh; // 节点句柄

    // 从参数服务器获取参数
    nh.param("/joy_to_cmd_vel_node/max_linear_speed", MAX_LINEAR_SPEED, 1.0);
    nh.param("/joy_to_cmd_vel_node/min_speed_threshold", MIN_SPEED_THRESHOLD, 0.1);
    nh.param("/joy_to_cmd_vel_node/wheel_distance", WHEEL_DISTANCE, 0.8);

    // 初始化发布者和订阅者
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Subscriber joy_sub = nh.subscribe("joy", 1000, joyCallback);

    ros::Rate rate(200); // 设置循环频率为200Hz

    // 主循环
    while (ros::ok()) {
        cmd_vel_pub.publish(cmd_vel); // 发布速度指令
        ros::spinOnce(); // 处理回调函数
        rate.sleep(); // 根据设置的频率休眠
    }

    return 0;
}



// 手柄控制消息的回调函数
// void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
//     // 使用非线性调整因子，平滑线速度和角速度的输入
//     double linear_scale = std::pow(fabs(msg->axes[1]), 0.5);
//     double angular_scale = std::pow(fabs(msg->axes[3]), 0.5);

//     // 应用缩放因子并乘以最大速度，独立控制线速度和角速度
//     double linear_vel = msg->axes[1] * linear_scale * MAX_LINEAR_SPEED;
//     double angular_vel = msg->axes[3] * angular_scale * (WHEEL_DISTANCE / 2); // 通过减小轮距因子来降低角速度的敏感度

//     geometry_msgs::Twist cmd_vel; // 创建Twist消息，用于发送速度指令

//     // 角速度控制的动态调整，确保在较高线速度时减小角速度
//     double dynamic_angular_limit = WHEEL_DISTANCE / (1 + 2 * fabs(linear_vel)); // 增加线速度依赖性，提高稳定性

//     // 应用安全限制，防止速度过高
//     cmd_vel.linear.x = std::min(linear_vel, MAX_LINEAR_SPEED);
//     cmd_vel.angular.z = std::min(angular_vel, dynamic_angular_limit);

//     // 避免微小的速度值引起不必要的移动
//     if (fabs(cmd_vel.linear.x) < MIN_SPEED_THRESHOLD) cmd_vel.linear.x = 0;
//     if (fabs(cmd_vel.angular.z) < MIN_SPEED_THRESHOLD) cmd_vel.angular.z = 0;

//     cmd_vel_pub.publish(cmd_vel); // 发布速度控制指令
// }
// void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
//     // 使用非线性调整因子，平滑线速度和角速度的输入
//     double linear_scale = std::pow(fabs(msg->axes[1]), 2);
//     double angular_scale = std::pow(fabs(msg->axes[3]), 2);

//     // 应用缩放因子并乘以最大速度，独立控制线速度和角速度
//     double linear_vel = -msg->axes[1] * linear_scale * MAX_LINEAR_SPEED; // 取反来调整线速度方向
//     double angular_vel = msg->axes[3] * angular_scale * (WHEEL_DISTANCE / 2); 
//     //double angular_vel = msg->axes[3] * angular_scale; 
//     // 如果手柄遥感回正，立即停止车辆
//     if (fabs(msg->axes[1]) < JOYSTICK_DEADZONE) {
//         linear_vel = 0.0;
//     }

//     // 角速度控制的动态调整，确保在较高线速度时减小角速度
//     double dynamic_angular_limit = WHEEL_DISTANCE / (1 + 2 * fabs(linear_vel)); 

//     // 控制角速度不至于太小
//     if (fabs(angular_vel) < MIN_ANGULAR_VELOCITY) {
//         angular_vel = copysign(MIN_ANGULAR_VELOCITY, angular_vel);
//     }

//     // 动态调整角速度的缩放因子和安全限制
//     double dynamic_angular_scale = std::pow(fabs(linear_vel) / MAX_LINEAR_SPEED, 2);
//     double dynamic_safe_angular_vel = std::min(angular_vel, dynamic_angular_limit * dynamic_angular_scale);

//     // 应用安全限制，防止速度过高
//     double safe_linear_vel = std::min(linear_vel, MAX_LINEAR_SPEED);
//     double safe_angular_vel = std::min(dynamic_safe_angular_vel, dynamic_angular_limit);

//     // 构造Twist消息
//     geometry_msgs::Twist cmd_vel; 
//     cmd_vel.linear.x = safe_linear_vel;
//     cmd_vel.angular.z = safe_angular_vel;

//     // 发布速度控制指令
//     cmd_vel_pub.publish(cmd_vel); 

//     // 加减速逻辑
//     if (msg->buttons[4] == 1) {  // L1 增加线速度
//         MAX_LINEAR_SPEED += 0.1;
//         MAX_LINEAR_SPEED = std::max(0.2, MAX_LINEAR_SPEED);  // 最小值为0.2
//         ROS_INFO("Max linear speed increased to: %f", MAX_LINEAR_SPEED);
//     }
//     if (msg->axes[2] == -1.0) {  // L2 减少线速度
//         MAX_LINEAR_SPEED -= 0.1;
//         MAX_LINEAR_SPEED = std::max(0.2, MAX_LINEAR_SPEED);  // 最小值为0.2
//         ROS_INFO("Max linear speed decreased to: %f", MAX_LINEAR_SPEED);
//     }
//     if (msg->buttons[5] == 1) {  // R1 增加角速度
//         MIN_ANGULAR_VELOCITY += 0.1;
//         MIN_ANGULAR_VELOCITY = std::max(0.2, MIN_ANGULAR_VELOCITY);  // 最小值为0.2
//         ROS_INFO("Min angular velocity increased to: %f", MIN_ANGULAR_VELOCITY);
//     }
//     if (msg->axes[5] == -1.0) {  // R2 减少角速度
//         MIN_ANGULAR_VELOCITY -= 0.1;
//         MIN_ANGULAR_VELOCITY = std::max(0.2, MIN_ANGULAR_VELOCITY);  // 最小值为0.2
//         ROS_INFO("Min angular velocity decreased to: %f", MIN_ANGULAR_VELOCITY);
//     }
// }