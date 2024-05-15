#if 1
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_test/CustomControlMsg.h> // 导入自定义消息类型

ros::Publisher custom_control_pub;
can_test::CustomControlMsg custom_msg;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // 获取手柄数据
    int x_axis = -joy->axes[6] * 2000; // X轴滑台需要运动的步数，乘以-1来矫正方向
    int y_axis = joy->axes[7] * 5000; // Y轴滑台需要运动的步数
    int button0 = joy->buttons[0];
    int button1 = joy->buttons[1];
    int button2 = joy->buttons[2];

    // 更新全局变量中的自定义消息数据
    custom_msg.axis6 = x_axis;
    custom_msg.axis7 = y_axis;
    custom_msg.button0 = button0;
    custom_msg.button1 = button1;
    custom_msg.button2 = button2;
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "joy_to_serial_node");
    ros::NodeHandle nh;

    // 创建一个Publisher，发布自定义消息
    custom_control_pub = nh.advertise<can_test::CustomControlMsg>("custom_control_topic", 1000);

    // 订阅joy话题
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);

    ros::Rate rate(200); // 设置循环频率为200Hz

    // 主循环
    while (ros::ok()) {
        custom_control_pub.publish(custom_msg); // 发布速度指令
        ros::spinOnce(); // 处理回调函数
        rate.sleep(); // 根据设置的频率休眠
    }

    return 0;
}

#endif
#if 0
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_test/CustomControlMsg.h> // 导入自定义消息类型

ros::Publisher custom_control_pub;
can_test::CustomControlMsg custom_msg;

// 手柄控制消息的回调函数
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    // 获取手柄数据
    int x_axis = -joy->axes[6] * 1000; // X轴滑台需要运动的步数，乘以-1来矫正方向
    int y_axis = joy->axes[7] * 1000; // Y轴滑台需要运动的步数
    int button0 = joy->buttons[0];
    int button1 = joy->buttons[1];
    int button2 = joy->buttons[2];

    // 更新全局变量中的自定义消息数据
    custom_msg.axis6 = x_axis;
    custom_msg.axis7 = y_axis;
    custom_msg.button0 = button0;
    custom_msg.button1 = button1;
    custom_msg.button2 = button2;

    // 打印调试信息
    ROS_INFO("Updated custom_msg: axis6=%d, axis7=%d, button0=%d, button1=%d, button2=%d",
             x_axis, y_axis, button0, button1, button2);
}

void timerCallback(const ros::TimerEvent&)
{
    // 发布自定义消息
    custom_control_pub.publish(custom_msg);

    // 打印调试信息
    ROS_INFO("Published custom_msg: axis6=%d, axis7=%d, button0=%d, button1=%d, button2=%d",
             custom_msg.axis6, custom_msg.axis7, custom_msg.button0, custom_msg.button1, custom_msg.button2);
}

int main(int argc, char** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "joy_to_serial_node");
    ros::NodeHandle nh;

    // 创建一个Publisher，发布自定义消息
    custom_control_pub = nh.advertise<can_test::CustomControlMsg>("custom_control_topic", 1000);

    // 订阅joy话题
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);

    // 创建定时器，每5毫秒（200Hz）调用一次回调函数
    ros::Timer timer = nh.createTimer(ros::Duration(0.005), timerCallback);

    // 进入ROS事件循环
    ros::spin();

    return 0;
}

#endif