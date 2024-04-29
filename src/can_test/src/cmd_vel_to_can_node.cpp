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

// 上一次发送的速度值，用于避免重复发送相同的速度值
int16_t last_left_speed = 0;
int16_t last_right_speed = 0;

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
    // 使用ROS的初始化函数来初始化节点，argc和argv分别代表命令行参数的数量和具体参数内容，"cmd_vel_to_can_node"是该节点的名称
    ros::init(argc, argv, "cmd_vel_to_can_node");
    
    // 创建一个NodeHandle对象nh，它允许节点与其他ROS节点通信、发布或订阅话题等
    ros::NodeHandle nh;

    // 使用nh.advertise()方法创建一个发布者（Publisher），用于发布CAN总线消息。
    // 消息类型为can_msgs::Frame，发布的主题名为"sent_messages"，队列大小设为100，以缓冲待发送的消息
    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);

    // 使用nh.subscribe()方法创建一个订阅者（Subscriber），订阅主题"cmd_vel"上的消息，
    // 当有新消息时，会调用cmdVelCallback回调函数进行处理，队列大小同样为10，用于暂存未处理的消息
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmdVelCallback);

    // 再创建一个订阅者，用于监听CAN总线接收到的消息，主题为"/received_messages"，
    // 当接收到新消息时，调用canCallBack回调函数处理这些消息，队列大小为100
    ros::Subscriber sub = nh.subscribe("/received_messages", 100, canCallBack);

    // 控制消息发布频率为10Hz
    ros::Rate rate(10);

    // 循环等待ROS节点终止
    while (ros::ok()) {
        ros::spinOnce(); // 处理回调函数
        rate.sleep(); // 控制发布频率
    }

    // 关闭ROS节点
    ros::shutdown();

    // 返回0表示程序成功执行完毕
    return 0;
}
