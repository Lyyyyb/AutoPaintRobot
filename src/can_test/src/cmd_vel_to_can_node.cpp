#include <ros/ros.h> // 导入ROS相关的头文件
#include <geometry_msgs/Twist.h> // 机器人运动控制消息头文件
#include <can_msgs/Frame.h> // CAN总线消息头文件
#include <cmath> // 导入数学计算相关头文件

// 定义常量
const double WHEEL_DISTANCE = 0.5; // 轮距，单位为米
const int16_t MAX_SPEED_VALUE = 32767; // 最大速度值，用于速度的数值转换
const uint8_t CAN_FRAME_DLC = 8; // CAN帧的数据长度

ros::Publisher can_pub; // 定义一个发布者，用于发送CAN消息

// 定义一个联合体，用于int16_t和两个uint8_t之间的数据转换
union SpeedData {
    int16_t speed; // 存储速度值
    uint8_t bytes[2]; // 存储速度值的字节形式
};

// 存储上一次发送的速度值，避免重复发送相同的速度
int16_t last_left_speed = 0;
int16_t last_right_speed = 0;

// 定义发布CAN帧的函数
void publishCanFrame(uint32_t id, uint8_t data[]) {
    can_msgs::Frame frame; // 创建一个CAN帧对象
    frame.id = id; // 设置帧的ID
    frame.dlc = CAN_FRAME_DLC; // 设置数据长度
    for (int i = 0; i < CAN_FRAME_DLC; ++i) {
        frame.data[i] = data[i]; // 填充数据到帧中
    }
    can_pub.publish(frame); // 发布CAN帧
}

// 处理cmd_vel话题消息的回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double v = msg->linear.x; // 读取线速度
    double omega = msg->angular.z; // 读取角速度
    double v_l = v - omega * WHEEL_DISTANCE / 2; // 计算左轮速度
    double v_r = v + omega * WHEEL_DISTANCE / 2; // 计算右轮速度

    SpeedData left_speed; // 左轮速度数据
    SpeedData right_speed; // 右轮速度数据

    left_speed.speed = static_cast<int16_t>(v_l * MAX_SPEED_VALUE); // 将左轮速度转换为int16_t类型
    right_speed.speed = static_cast<int16_t>(v_r * MAX_SPEED_VALUE); // 将右轮速度转换为int16_t类型

    // 如果左右轮速度没有变化，则不发送CAN帧
    if (left_speed.speed == last_left_speed && right_speed.speed == last_right_speed) {
        return;
    }

    // 更新上一次发送的速度值
    last_left_speed = left_speed.speed;
    last_right_speed = right_speed.speed;

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
    // 初始化ROS节点
    ros::init(argc, argv, "cmd_vel_to_can_node");

    // 创建NodeHandle对象，允许节点与ROS系统通信
    ros::NodeHandle nh;

    // 创建一个发布者对象，用于发送CAN消息
    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);

    // 创建一个订阅者对象，用于接收cmd_vel消息，并指定回调函数
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);

    // 创建另一个订阅者对象，用于接收CAN消息
    ros::Subscriber sub = nh.subscribe("/received_messages", 100, canCallBack);

    // 设置循环频率
    ros::Rate rate(10);

    // ROS节点循环，等待消息或事件
    while (ros::ok()) {
        ros::spinOnce(); // 处理一次回调函数
        rate.sleep(); // 控制循环的时间间隔
    }

    // 关闭ROS节点
    ros::shutdown();

    // 程序结束
    return 0;
}
