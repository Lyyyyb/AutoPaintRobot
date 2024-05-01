#include "tank_control/my_robot_controller.h" 

float wheel_separation_;
float wheel_radius_;
int16_t left_wheel_velocity_;
int16_t right_wheel_velocity_;

// 速度命令回调
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    // 获取线速度和角速度
    float linear_speed = msg->linear.x;
    float angular_speed = msg->angular.z;

    //std::cout << "linear_speed: " << linear_speed << ", angular_speed: " << angular_speed << std::endl;

    // 根据线速度和角速度计算左右轮的速度
    left_wheel_velocity_ = static_cast<int16_t>((linear_speed - (angular_speed * wheel_separation_ / 2)) / wheel_radius_);
    right_wheel_velocity_ = static_cast<int16_t>((linear_speed + (angular_speed * wheel_separation_ / 2)) / wheel_radius_);

    //ROS_INFO("left_wheel_velocity: %d, right_wheel_velocity: %d", left_wheel_velocity_, right_wheel_velocity_);
    // MyRobotHardware::robot_hardware_->getMotorSpeed();
    // 设置硬件接口中的速度命令
    MyRobotHardware::setMotorSpeed(left_wheel_velocity_, right_wheel_velocity_);
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "tank_control_node");  // 初始化 ROS 节点
    ros::NodeHandle nh;  // 创建节点句柄

    if (!nh.param<float>("tank_control/wheel_separation", wheel_separation_, 0.5)) {            
        throw std::runtime_error("Failed to get 'wheel_separation' parameter");
    }

    if (!nh.param<float>("tank_control/wheel_radius", wheel_radius_, 0.1)) {            
        throw std::runtime_error("Failed to get 'wheel_radius' parameter");
    }

    can_pub_ = nh.advertise <can_msgs::Frame>("sent_messages", 100);  // 创建 CAN 发布器
    cmd_vel_sub_ = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 10, cmdVelCallback);  // 创建速度命令订阅器
    //can_sub_ = nh.subscribe("received_messages", 100, CANInterfaceManager::frameCallback);  // 创建 CAN 订阅器

    //MyRobotHardware::initMotor();  // 初始化电机
    
    ros::spin();  // 进入 ROS 主循环
    return 0;  // 退出
}
