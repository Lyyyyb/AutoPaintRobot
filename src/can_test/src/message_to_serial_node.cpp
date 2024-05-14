#include <ros/ros.h>
#include <can_test/CustomControlMsg.h>
#include <serial/serial.h>
#include <sstream>

serial::Serial ser;
int pump_duty = 0;
int last_pump_duty = -1; // 用于保存上一次的水泵占空比

void customControlCallback(const can_test::CustomControlMsg::ConstPtr& msg)
{
    // 获取自定义消息中的数据
    int x_axis = msg->axis6;
    int y_axis = msg->axis7;
    int button0 = msg->button0;
    int button1 = msg->button1;
    int button2 = msg->button2;

    int speed = 10000; // 默认速度
    int mode = 1; // 默认模式

    // 检查是否需要发送滑台控制指令
    if (x_axis != 0 || y_axis != 0) {
        // 构建并发送滑台控制指令
        std::ostringstream move_cmd;
        move_cmd << "move " << x_axis << " " << y_axis << " " << speed << " " << mode << "\r\n";
        std::string move_cmd_str = move_cmd.str();
        try {
            ser.write(move_cmd_str);
            ROS_INFO_STREAM("Sent move command: " << move_cmd_str); // 打印发送的移动指令
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Failed to send move command: " << e.what());
            if (!ser.isOpen()) {
                ROS_ERROR_STREAM("Reopening serial port...");
                try {
                    ser.open();
                    ROS_INFO_STREAM("Serial Port reopened");
                } catch (const std::exception& e) {
                    ROS_ERROR_STREAM("Failed to reopen serial port: " << e.what());
                }
            }
        }
    }

    // 控制水泵
    if (button0 == 1) {
        // 停止水泵
        pump_duty = 0;
    } else if (button2 == 1) {
        // 增加水泵占空比
        pump_duty = std::min(pump_duty + 10, 100); // 最大占空比为100
    } else if (button1 == 1) {
        // 减少水泵占空比
        pump_duty = std::max(pump_duty - 10, 0); // 最小占空比为0
    }

    // 只有在水泵占空比发生变化时才发送水泵控制指令
    if (pump_duty != last_pump_duty) {
        // 构建并发送水泵控制指令
        std::ostringstream pump_cmd;
        pump_cmd << "pump " << pump_duty;
        std::string pump_cmd_str = pump_cmd.str();
        try {
            ser.write(pump_cmd_str + "\r\n");
            ROS_INFO_STREAM("Sent pump command: " << pump_cmd_str); // 打印发送的水泵指令
            last_pump_duty = pump_duty; // 更新上一次的水泵占空比
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("Failed to send pump command: " << e.what());
            if (!ser.isOpen()) {
                ROS_ERROR_STREAM("Reopening serial port...");
                try {
                    ser.open();
                    ROS_INFO_STREAM("Serial Port reopened");
                } catch (const std::exception& e) {
                    ROS_ERROR_STREAM("Failed to reopen serial port: " << e.what());
                }
            }
        }
    }
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "serial_publisher_node");
    ros::NodeHandle nh;

    // 设置串口参数
    ser.setPort("/dev/ttyUSB0"); // 请根据实际串口号修改
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);

    // 打开串口
    try {
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    // 订阅自定义消息话题
    ros::Subscriber sub = nh.subscribe<can_test::CustomControlMsg>("custom_control_topic", 1000, customControlCallback);

    // ROS循环
    ros::Rate rate(200); // 设置循环频率
    while (ros::ok()) {
        ros::spinOnce(); // 处理回调函数
        rate.sleep(); // 控制循环频率
    }

    return 0;
}
