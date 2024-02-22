#include "robot_bringup/robot.h"
#include <dynamic_reconfigure/server.h>
#include "robot_bringup/speedparamConfig.h"
#include "ros/ros.h"
#include "mbot_linux_serial.h"

double RobotV_ = 0;
double RobotYawRate_ = 0;
int speedparam_x;
int speedparam_y;


/* 传递速度系数参数 */
namespace robot_bringup{//必须是包名
    void speedcallback(speedparamConfig &config,uint32_t level)
    {
        speedparam_x = config.speedparam_x;
        speedparam_y = config.speedparam_y;
        ROS_INFO("success to speedcallback");

    }

}

/* 速度控制消息的回调函数 */
void cmdCallback(const geometry_msgs::Twist& msg)
{
    RobotV_  = msg.linear.x*(-1)*speedparam_x; //msg.linear.x初始值是0.5//param is 1125
    RobotYawRate_ = msg.angular.z*speedparam_y;//param is 4650
    
    ROS_INFO("success to cmdCallback");

}
    
int main(int argc, char** argv)
{
    //初始化ROS节点
    ros::init(argc, argv, "mbot_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");
    robot::robot myrobot;

    //serialInit();
    
    // 通过param获取参数
    nhPrivate.param("speedparam_x",speedparam_x,1125);
    nhPrivate.param("speedparam_y",speedparam_y,4655);
    
    //创建动态参数服务，并绑定回调函数
    dynamic_reconfigure::Server<robot_bringup::speedparamConfig> server;
    dynamic_reconfigure::Server<robot_bringup::speedparamConfig>::CallbackType ser = boost::bind(&robot_bringup::speedcallback,_1,_2);
    server.setCallback(ser);

    //初始化robot
    if(!myrobot.init())
        ROS_ERROR("myrobot initialized failed.");
        
    ROS_INFO("myrobot initialized successful.");
    // ROS_INFO("speedparam_x %d",speedparam_x);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 50, cmdCallback);

    //循环运行
    ROS_INFO("start to loop");
    ros::Rate loop_rate(10);
    while (ros::ok()) 
    {
        ros::spinOnce();
        
        //机器人控制RobotV_,RobotYawRate_这两个参数传入到robot.cpp/bool robot::deal，再传入到mbot_linux_serial.cpp中
        //通过deal对RobotV_和RobotYawRate_进行判定电机的转动和停止
        myrobot.deal(RobotV_,RobotYawRate_);//传入到robot.cpp文件中 robot::deal()
        
        loop_rate.sleep();
    }

    return 0;
}

