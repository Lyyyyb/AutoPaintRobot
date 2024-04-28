#include <ros/ros.h> // ROS相关头文件
#include <geometry_msgs/Twist.h> // 机器人运动控制消息头文件
#include <can_msgs/Frame.h> // CAN消息头文件
#include <cmath> // 数学计算头文件

// 常量定义
const double WHEEL_DISTANCE = 0.5; // 车轮之间的距离，单位为米
const int16_t MAX_SPEED_VALUE = 32767; // 最大速度值，用于速度转换
const uint8_t CAN_FRAME_DLC = 8; // CAN帧的数据长度码

ros::Publisher can_pub; // CAN消息发布者

// 定义一个联合体，用于int16_t和两个uint8_t之间的转换
union SpeedData {
    int16_t speed; // 速度值
    uint8_t bytes[2]; // 两个字节的字节数组
};

// 发布CAN帧函数
void publishCanFrame(uint32_t id, uint8_t data[]) {
    can_msgs::Frame frame; // 创建CAN帧对象
    frame.id = id; // 设置帧ID
    frame.dlc = CAN_FRAME_DLC; // 设置数据长度码
    for (int i = 0; i < CAN_FRAME_DLC; ++i) {
        frame.data[i] = data[i]; // 填充数据
    }
    can_pub.publish(frame); // 发布CAN帧
}

// cmd_vel话题回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double v = msg->linear.x; // 获取线速度
    double omega = msg->angular.z; // 获取角速度
    double v_l = v - omega * WHEEL_DISTANCE / 2; // 计算左轮速度
    double v_r = v + omega * WHEEL_DISTANCE / 2; // 计算右轮速度

    SpeedData left_speed; // 左轮速度数据
    SpeedData right_speed; // 右轮速度数据

    left_speed.speed = static_cast<int16_t>(v_l * MAX_SPEED_VALUE); // 将左轮速度转换为int16_t类型
    right_speed.speed = static_cast<int16_t>(v_r * MAX_SPEED_VALUE); // 将右轮速度转换为int16_t类型

    // 构造左轮和右轮的CAN数据数组
    uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_speed.bytes[0], left_speed.bytes[1], 0x00, 0x00};
    uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_speed.bytes[0], right_speed.bytes[1], 0x00, 0x00};

    publishCanFrame(0x601, data_l); // 发布左轮CAN帧
    publishCanFrame(0x602, data_r); // 发布右轮CAN帧
}

// CAN消息回调函数
void canCallBack(const can_msgs::Frame::ConstPtr& msg) {
    // 这里可以添加处理接收到的CAN消息的代码
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_to_can_node");
    ros::NodeHandle nh;

    // Advertise the CAN message publisher
    can_pub = nh.advertise <can_msgs::Frame>("sent_messages", 100);
    // Subscribe to the cmd_vel topic
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);

    ros::Subscriber sub = nh.subscribe("/received_messages", 100, canCallBack);

    ros::spin();
	return 0;
}