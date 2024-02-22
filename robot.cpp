#include <vector>
#include "robot_bringup/robot.h"
#include "robot_bringup/mbot_linux_serial.h"
//#include <boost/bind.hpp>
//#include <boost/asio.hpp>
using namespace std;

namespace robot
{
    robot::robot():x_(0.0), y_(0.0), th_(0.0),vx_(0.0), vy_(0.0), vth_(0.0),sensFlag_(0),receFlag_(0) {}//构造函数
    robot::~robot(){}            //析构函数

    /********************************************************
    函数功能：串口参数初始化、时间变量初始化、实例化发布对象
    入口参数：无
    出口参数：bool
    ********************************************************/
    bool robot::init()
    {
        // 串口初始化连接
        serialInit();
               
        ros::Time::init();
        current_time_ = ros::Time::now();
        last_time_ = ros::Time::now();	
        
        return true;
    }
 
    /********************************************************
    函数功能：对电机转动和停止进行判定
    入口参数：RobotV,RobotYawRat
    出口参数：StopFlag
    ********************************************************/
    void robot::deal(double RobotV, double RobotYawRate)
    {
        unsigned char StopFlag;
        if(RobotV==0 && RobotYawRate==0)//电机正常停止模式
        {
            StopFlag=0x02;
        }else
        {
            StopFlag=0x01;//停止
        }
        writeSpeed(RobotV,RobotYawRate,StopFlag);
        ROS_INFO("RobotV RobotYawRate : %f %f",RobotV,RobotYawRate);
    }
}

