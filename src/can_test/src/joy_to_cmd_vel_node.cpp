#include <ros/ros.h> 
#include <sensor_msgs/Joy.h> 
#include <geometry_msgs/Twist.h> 
#include <boost/bind.hpp> 
#include <functional> 
#include <cmath> // 添加数学函数头文件


// 常量定义
double MAX_LINEAR_SPEED = 1.0; // 定义最大线速度
double MIN_SPEED_THRESHOLD = 0.1; // 定义速度的最小阈值，低于此速度不进行处理
double WHEEL_DISTANCE = 0.8; // 轮距，用于计算转弯时的角速度
double JOYSTICK_DEADZONE = 0.05; // 手柄的死区，可以根据实际情况进行调整
double MIN_ANGULAR_VELOCITY = 0.1; // 最小角速度
double scale_factor = 0.5; // 可以根据需要调整这个值
ros::Publisher cmd_vel_pub; // 定义一个发布者，用来发送速度控制指令
double linear_scale_exponent = 0.5; // 添加线速度非线性调整因子
double angular_scale_exponent = 0.5; // 添加角速度非线性调整因子
double speed_smoothing_factor = 0.2; // 添加速度平滑处理因子
// 手柄控制消息的回调函数
// void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
//     ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]); // 打印线速度和角速度信息

//     double linear_vel = -msg->axes[1] * scale_factor; // 应用缩放因子
//     double angular_vel = msg->axes[3] ; // 应用缩放因子xxzx
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
void joyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
    // 使用非线性调整因子，平滑线速度和角速度的输入
    double linear_scale = std::pow(fabs(msg->axes[1]), 2);
    double angular_scale = std::pow(fabs(msg->axes[3]), 2);

    // 应用缩放因子并乘以最大速度，独立控制线速度和角速度
    double linear_vel = -msg->axes[1] * linear_scale * MAX_LINEAR_SPEED; // 取反来调整线速度方向
    double angular_vel = msg->axes[3] * angular_scale * (WHEEL_DISTANCE / 2); 

    // 如果手柄遥感回正，立即停止车辆
    if (fabs(msg->axes[1]) < JOYSTICK_DEADZONE) {
        linear_vel = 0.0;
    }

    // 角速度控制的动态调整，确保在较高线速度时减小角速度
    double dynamic_angular_limit = WHEEL_DISTANCE / (1 + 2 * fabs(linear_vel)); 

    // 控制角速度不至于太小
    if (fabs(angular_vel) < MIN_ANGULAR_VELOCITY) {
        angular_vel = copysign(MIN_ANGULAR_VELOCITY, angular_vel);
    }

    // 动态调整角速度的缩放因子和安全限制
    double dynamic_angular_scale = std::pow(fabs(linear_vel) / MAX_LINEAR_SPEED, 2);
    double dynamic_safe_angular_vel = std::min(angular_vel, dynamic_angular_limit * dynamic_angular_scale);

    // 应用安全限制，防止速度过高
    double safe_linear_vel = std::min(linear_vel, MAX_LINEAR_SPEED);
    double safe_angular_vel = std::min(dynamic_safe_angular_vel, dynamic_angular_limit);

    // 构造Twist消息
    geometry_msgs::Twist cmd_vel; 
    cmd_vel.linear.x = safe_linear_vel;
    cmd_vel.angular.z = safe_angular_vel;

    // 发布速度控制指令
    cmd_vel_pub.publish(cmd_vel); 

    // 加减速逻辑
    if (msg->buttons[4] == 1) {  // L1 增加线速度
        MAX_LINEAR_SPEED += 0.1;
        MAX_LINEAR_SPEED = std::max(0.2, MAX_LINEAR_SPEED);  // 最小值为0.2
        ROS_INFO("Max linear speed increased to: %f", MAX_LINEAR_SPEED);
    }
    if (msg->axes[2] == -1.0) {  // L2 减少线速度
        MAX_LINEAR_SPEED -= 0.1;
        MAX_LINEAR_SPEED = std::max(0.2, MAX_LINEAR_SPEED);  // 最小值为0.2
        ROS_INFO("Max linear speed decreased to: %f", MAX_LINEAR_SPEED);
    }
    if (msg->buttons[5] == 1) {  // R1 增加角速度
        MIN_ANGULAR_VELOCITY += 0.1;
        MIN_ANGULAR_VELOCITY = std::max(0.2, MIN_ANGULAR_VELOCITY);  // 最小值为0.2
        ROS_INFO("Min angular velocity increased to: %f", MIN_ANGULAR_VELOCITY);
    }
    if (msg->axes[5] == -1.0) {  // R2 减少角速度
        MIN_ANGULAR_VELOCITY -= 0.1;
        MIN_ANGULAR_VELOCITY = std::max(0.2, MIN_ANGULAR_VELOCITY);  // 最小值为0.2
        ROS_INFO("Min angular velocity decreased to: %f", MIN_ANGULAR_VELOCITY);
    }
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