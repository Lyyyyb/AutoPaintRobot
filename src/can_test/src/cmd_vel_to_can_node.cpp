#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <boost/bind.hpp>

// 变量声明
double wheel_distance; // 轮距，单位为米
double max_speed_value; // 最大速度值，用于速度的数值转换
int can_frame_dlc; // CAN帧的数据长度
int can_id_left_wheel; // 左轮CAN ID
int can_id_right_wheel; // 右轮CAN ID
int loop_rate; // 循环频率

ros::Publisher can_pub; // 定义一个发布者，用于发送CAN消息

// 定义一个联合体，用于int16_t和两个uint8_t之间的数据转换
union SpeedData {
    int16_t value; // 存储速度值
    uint8_t bytes[2]; // 存储速度值的字节形式
};

// 存储上一次发送的速度值，避免重复发送相同的速度
int16_t last_left_speed = 0;
int16_t last_right_speed = 0;

// 电机初始化状态
bool left_motor_initialized = false;
bool right_motor_initialized = false;
// 定义发布CAN帧的函数
// 修改can发布检查
// 修改发布CAN帧的函数，加入发布者参数
void publishCanFrame(int id, uint8_t data[], ros::Publisher& pub) {
    can_msgs::Frame frame;
    frame.id = id;
    frame.dlc = can_frame_dlc;
    for (int i = 0; i < can_frame_dlc; ++i) {
        frame.data[i] = data[i];
    }
    // 使用传入的发布者对象进行发布
    pub.publish(frame);
}
// 处理cmd_vel话题消息的回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher& pub) {
    double v = msg->linear.x; // 读取线速度
    double omega = msg->angular.z; // 读取角速度

    // 如果线速度和角速度都为零，则停止车辆
    if (v == 0 && omega == 0) {
        // 停止车辆，发送零速度命令
        SpeedData stop_speed;
        stop_speed.value = 0;

        uint8_t stop_data[] = {0x2B, 0x01, 0x20, 0x00, stop_speed.bytes[0], stop_speed.bytes[1], 0x00, 0x00};

        publishCanFrame(can_id_left_wheel, stop_data, pub); // 发布左轮CAN帧，传入发布者对象
        publishCanFrame(can_id_right_wheel, stop_data, pub); // 发布右轮CAN帧，传入发布者对象

        return; // 停止后不再继续执行后面的代码
    }

    double v_l = v - omega * wheel_distance / 2; // 计算左轮速度
    double v_r = v + omega * wheel_distance / 2; // 计算右轮速度

    // 转换速度到占空比，范围 -1000 到 1000
    int duty_cycle_left = static_cast<int>((v_l / max_speed_value) * 1000);
    int duty_cycle_right = static_cast<int>((v_r / max_speed_value) * 1000);
    duty_cycle_left = std::max(std::min(duty_cycle_left, 1000), -1000);
    duty_cycle_right = std::max(std::min(duty_cycle_right, 1000), -1000);

    // 日志输出当前占空比
    ROS_INFO("Left wheel duty cycle: %d, Right wheel duty cycle: %d", duty_cycle_left, duty_cycle_right);

    // 如果左右轮占空比没有变化，则不发送CAN帧
    if (duty_cycle_left == last_left_speed && duty_cycle_right == last_right_speed) {
        return;
    }

    SpeedData left_duty_cycle, right_duty_cycle;
    left_duty_cycle.value = duty_cycle_left;
    right_duty_cycle.value = duty_cycle_right;

    // 更新上一次发送的占空比
    last_left_speed = duty_cycle_left;
    last_right_speed = duty_cycle_right;

    // 构造左轮和右轮的CAN数据数组
    uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_duty_cycle.bytes[0], left_duty_cycle.bytes[1], 0x00, 0x00};
    uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_duty_cycle.bytes[0], right_duty_cycle.bytes[1], 0x00, 0x00};

    publishCanFrame(can_id_left_wheel, data_l, pub); // 发布左轮CAN帧，传入发布者对象
    publishCanFrame(can_id_right_wheel, data_r, pub); // 发布右轮CAN帧，传入发布者对象
}



// 初始化电机函数
void initMotors(ros::Publisher& pub) {
    uint8_t init_motor_data[] = {0x2f, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
    int max_attempts = 10;
    int attempts = 0;

    while (!(left_motor_initialized && right_motor_initialized) && attempts < max_attempts) {
        publishCanFrame(0x601, init_motor_data, pub);
        publishCanFrame(0x602, init_motor_data, pub);
        ros::spinOnce(); // 确保处理CAN帧的回调函数
        ros::Duration(0.5).sleep();
        attempts++;
        ROS_INFO("Attempt %d to initialize motors", attempts);

        if (left_motor_initialized && right_motor_initialized) {
            ROS_INFO("Both motors initialized successfully.");
            break; // 如果两个电机都已初始化，立即退出循环
        }
    }

    if (!left_motor_initialized || !right_motor_initialized) {
        ROS_ERROR("Failed to initialize motors after %d attempts. Continuing with limited functionality.", attempts);
    }
}




// CAN消息回调函数
void canCallBack(const can_msgs::Frame::ConstPtr& msg) {
    switch (msg->id) {
        case 0x581:
        case 0x582:
            // 确保data[0]为0x60, data[1]为0x00, 和data[2]为0x20
            if (msg->data[0] == 0x60 && msg->data[1] == 0x00 && msg->data[2] == 0x20) {
                bool isLeft = (msg->id == 0x581);
                if (isLeft) {
                    left_motor_initialized = true;
                    ROS_INFO("Left motor initialized successfully!");
                } else {
                    right_motor_initialized = true;
                    ROS_INFO("Right motor initialized successfully!");
                }
            } else {
                ROS_WARN("Unexpected data in known frame 0x%X: %02X %02X %02X", msg->id, msg->data[0], msg->data[1], msg->data[2]);
            }
            break;

        default:
            ROS_WARN("Ignoring unexpected CAN frame with ID: 0x%X, data: %02X %02X %02X %02X %02X %02X %02X %02X",
                     msg->id, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
            break;
    }
}





int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_to_can_node");
    ros::NodeHandle nh;

    // 设置参数
    // 设置并验证参数
    nh.param("wheel_distance", wheel_distance, 0.8); // 默认轮距为0.5米
    nh.param("max_speed_value", max_speed_value, 1.0); // 最大速度设置为1.0 m/s
    nh.param("can_frame_dlc", can_frame_dlc, 8); // 默认CAN帧长度为8
    nh.param("can_id_left_wheel", can_id_left_wheel, 0x601); // 左轮的CAN ID
    nh.param("can_id_right_wheel", can_id_right_wheel, 0x602); // 右轮的CAN ID
    nh.param("loop_rate", loop_rate, 10); // 循环频率设置为10Hz


    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 100, boost::bind(cmdVelCallback, _1, boost::ref(can_pub)));
    ros::Subscriber sub = nh.subscribe("/received_messages", 100, canCallBack);

    // 初始化电机
    initMotors(can_pub);

    ros::Rate rate(loop_rate);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
