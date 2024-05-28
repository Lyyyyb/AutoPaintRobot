#if 1
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <sstream>

// 串口配置
serial::Serial ser;

// 初始化水泵占空比
int pump_duty = 0;

// 回调函数，用于解析joy话题数据并通过串口发送控制指令
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // 获取手柄数据
    int x_axis = -joy->axes[6] * 1000; // X轴滑台需要运动的步数，乘以-1来矫正方向
 // X轴滑台需要运动的步数
    int y_axis = joy->axes[7] * 1000; // Y轴滑台需要运动的步数
    int speed = 1000; // 默认速度
    int mode = 1; // 默认模式

    // 检查是否需要发送滑台控制指令
    if (x_axis != 0 || y_axis != 0) {
        // 构建并发送滑台控制指令
        std::ostringstream move_cmd;
        move_cmd << "move " << x_axis << " " << y_axis << " " << speed << " " << mode << "\r\n";
        std::string move_cmd_str = move_cmd.str();
        ser.write(move_cmd_str);
        ROS_INFO_STREAM("Sent move command: " << move_cmd_str); // 打印发送的移动指令
    }

    // 控制水泵
    if (joy->buttons[0] == 1) {
        // 停止水泵
        pump_duty = 0;
    } else if (joy->buttons[2] == 1) {
        // 增加水泵占空比
        pump_duty = std::min(pump_duty + 10, 100); // 最大占空比为100
    } 
    // else if (joy->buttons[1] == 1) {
    //     // 减少水泵占空比
    //     pump_duty = std::max(pump_duty - 10, 0); // 最小占空比为0
    // }

    // 构建并发送水泵控制指令
    std::ostringstream pump_cmd;
    pump_cmd << "pump " << pump_duty;
    std::string pump_cmd_str = pump_cmd.str();
    ser.write(pump_cmd_str + "\r\n");
    ROS_INFO_STREAM("Sent pump command: " << pump_cmd_str); // 打印发送的水泵指令
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "joy_to_serial_node");
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
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    // 订阅joy话题
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    // ROS循环
    ros::spin();

    return 0;
}
#endif
#if 0
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>
#include <sstream>

// 串口配置
serial::Serial ser;

// 初始化水泵占空比
int pump_duty = 0;

// 控制滑台移动的数据
int x_axis = 0;
int y_axis = 0;

// 回调函数，解析joy话题数据
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    x_axis = -joy->axes[6] * 1000; // X轴滑台需要运动的步数，乘以-1来矫正方向
    y_axis = joy->axes[7] * 1000; // Y轴滑台需要运动的步数

    // 控制水泵
    if (joy->buttons[0] == 1) {
        // 停止水泵
        pump_duty = 0;
    } else if (joy->buttons[2] == 1) {
        // 增加水泵占空比
        pump_duty = std::min(pump_duty + 10, 100); // 最大占空比为100
    } else if (joy->buttons[1] == 1) {
        // 减少水泵占空比
        pump_duty = std::max(pump_duty - 10, 0); // 最小占空比为0
    }
    // 发送水泵控制指令
    std::ostringstream pump_cmd;
    pump_cmd << "pump " << pump_duty << "\r\n";
    std::string pump_cmd_str = pump_cmd.str();
    ser.write(pump_cmd_str);
    ROS_INFO_STREAM("Sent pump command: " << pump_cmd_str); // 打印发送的水泵指令
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "joy_to_serial_node");
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
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
    } else {
        return -1;
    }

    // 订阅joy话题
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    // ROS循环
    ros::Rate loop_rate(10); // 10Hz
    while (ros::ok()) {
        ros::spinOnce();

        // 发送滑台控制指令
        if (x_axis != 0 || y_axis != 0) {
            int speed = 1000; // 默认速度
            int mode = 1; // 默认模式

            // 构建并发送滑台控制指令
            std::ostringstream move_cmd;
            move_cmd << "move " << x_axis << " " << y_axis << " " << speed << " " << mode << "\r\n";
            std::string move_cmd_str = move_cmd.str();
            ser.write(move_cmd_str);
            ROS_INFO_STREAM("Sent move command: " << move_cmd_str); // 打印发送的移动指令

            // 指令发送后，重置滑台移动数据
            x_axis = 0;
            y_axis = 0;
        }



        loop_rate.sleep();
    }

    return 0;
}
#endif