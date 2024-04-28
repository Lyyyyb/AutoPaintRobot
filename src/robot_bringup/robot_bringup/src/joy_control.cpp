//#include "robot_bringup/robot.h"
//#include <dynamic_reconfigure/server.h>
#include "ros/ros.h"
//#include "mbot_linux_serial.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

int i=100; 

class JoyControl
{
private:
    // 处理手柄发送过来的信息
    void callback(const sensor_msgs::Joy::ConstPtr &joy);
    // 实例化ROS句柄
    ros::NodeHandle nh;
    // 定义订阅者对象，用来订阅手柄发送的数据
    ros::Subscriber sub;
    // 定义发布者对象，用来将手柄数据发布到乌龟控制话题上
    ros::Publisher pub;
    // 用来接收launch文件中设置的参数，绑定手柄摇杆、轴的映射
    int axis_linear, axis_angular;
    //用于接收joy参数的基础值：axis_linear,axis_angular,bu
    float firstdata[2];
    float finaldate[2];

public:
    JoyControl();
};

JoyControl::JoyControl()
{
    // 从参数服务器读取的参数
    nh.param<int>("axis_linear", axis_linear, 1);
    nh.param<int>("axis_angular", axis_angular, 3);
 
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyControl::callback, this);

}

void JoyControl::callback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist v;
    /* 将手柄摇杆轴拨动时值的输出赋值的线速度和角速度，在这个地方可用参数服务器，也可不用 */
    // basedata[0]=joy->axes[1];
    // basedata[1]=joy->axes[3];
    firstdata[0]=joy->axes[axis_linear];
    firstdata[1]=joy->axes[axis_angular];
    /* buttons就不用参数服务器了 */
    //basedata[2]=joy->buttons[5];
    //basedata[3]=joy->buttons[6];

    /* 按键左加右减，以百分比的形式计算 */
    if(joy->buttons[4]==1)//增加
    {
        i=i+10;
    }
    if(joy->buttons[5]==1)//减小
    {
        i=i-10;
    }
    finaldate[0]=firstdata[0]*i*0.01;
    finaldate[1]=firstdata[1]*i*0.01;

    v.linear.x = finaldate[0]/4;
    v.angular.z = finaldate[1]/2;

    ROS_INFO("线速度:%.3lf ; 角速度:%.3lf", v.linear.x, v.angular.z);
    pub.publish(v);
}

int main(int argc, char **argv)
{
    // 设置编码
    setlocale(LC_ALL, "");
    // 初始化ROS节点
    ros::init(argc, argv, "joy_control");
    //在JoyControl这个前面不能加东西
    JoyControl joycontrol;

    ros::Rate loop_rate(10);

    ros::spin(); 
    return 0;
}

