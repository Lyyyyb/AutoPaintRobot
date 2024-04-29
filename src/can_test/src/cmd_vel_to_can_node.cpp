#include <ros/ros.h> // 导入ROS相关的头文件
#include <geometry_msgs/Twist.h> // 机器人运动控制消息头文件
#include <can_msgs/Frame.h> // CAN总线消息头文件
#include <cmath> // 导入数学计算相关头文件

// 变量声明
double wheel_distance; // 轮距，单位为米
int max_speed_value; // 最大速度值，用于速度的数值转换
int can_frame_dlc; // CAN帧的数据长度
int can_id_left_wheel; // 左轮CAN ID
int can_id_right_wheel; // 右轮CAN ID
int loop_rate; // 循环频率

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
// 修改can发布检查
void publishCanFrame(int id, uint8_t data[]) {
    can_msgs::Frame frame;
    frame.id = id;
    frame.dlc = can_frame_dlc;
    for (int i = 0; i < can_frame_dlc; ++i) {
        frame.data[i] = data[i];
    }
    // 直接发布，不进行订阅者检查
    can_pub.publish(frame);
}
// 处理cmd_vel话题消息的回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    double v = msg->linear.x; // 读取线速度
    double omega = msg->angular.z; // 读取角速度
    double v_l = v - omega * wheel_distance / 2; // 计算左轮速度
    double v_r = v + omega * wheel_distance / 2; // 计算右轮速度

    SpeedData left_speed; // 左轮速度数据
    SpeedData right_speed; // 右轮速度数据

    // 速度缩放并转换为int16_t类型，同时处理溢出
    left_speed.speed = std::max(std::min(static_cast<int>(v_l * max_speed_value), 32767), -32768);
    right_speed.speed = std::max(std::min(static_cast<int>(v_r * max_speed_value), 32767), -32768);

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

    publishCanFrame(can_id_left_wheel, data_l); // 发布左轮CAN帧
    publishCanFrame(can_id_right_wheel, data_r); // 发布右轮CAN帧
}


// CAN消息回调函数
void canCallBack(const can_msgs::Frame::ConstPtr& msg) {
    // 这里可以添加处理接收到的CAN消息的代码
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_to_can_node");
    ros::NodeHandle nh;

    // 获取参数
    nh.param("wheel_distance", wheel_distance, 0.5);
    nh.param("max_speed_value", max_speed_value, 32767);
    nh.param("can_frame_dlc", can_frame_dlc, 8);
    nh.param("can_id_left_wheel", can_id_left_wheel, 0x601);
    nh.param("can_id_right_wheel", can_id_right_wheel, 0x602);
    nh.param("loop_rate", loop_rate, 10);

    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 100, cmdVelCallback);
    ros::Subscriber sub = nh.subscribe("/received_messages", 100, canCallBack);

    ros::Rate rate(loop_rate);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}