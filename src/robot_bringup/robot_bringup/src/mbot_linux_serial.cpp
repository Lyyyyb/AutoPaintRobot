#include "robot_bringup/mbot_linux_serial.h"
#include "string.h"
#include <boost/bind.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial/serial.h"

using namespace std;
using namespace boost::asio;

serial::Serial ser;
#define sBUFFER_SIZE 1024
#define rBUFFER_SIZE 1024
unsigned char s_buffer[sBUFFER_SIZE];
unsigned char r_buffer[rBUFFER_SIZE];

//串口相关对象
//boost::asio::io_service iosev;
//boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
//boost::system::error_code err;

/********************************************************
            串口发送接收相关常量、变量、共用体对象
********************************************************/
const unsigned char ender = {0x0D};
const unsigned char header = {0xAA};

//驱动模式
struct Control_Way
{
    unsigned char data[1];
} VelWay;

//发送左右轮速控制速度共用体,传感器的X，Z，Angle，用于数据转化
union sendData1
{
    short l;//short是短整形、两字节，16位
    unsigned char data1[2];//data[2]储存两个十六进制数，共16位。//所以两者可以转化
} leftVelSet;

union sendData2
{
    short r;//short是短整形、两字节，16位
    unsigned char data2[2];//data[2]储存两个十六进制数，共16位。//所以两者可以转化
} rightVelSet;

const double ROBOT_LENGTH = 0.85; // 两轮之间距离m
const double ROBOT_RADIUS = 0.10;  //  轮子半径m

/********************************************************
函数功能：串口参数初始化
入口参数：无
出口参数：
函数引用：robot.cpp中的bool robot::init(){}引用
********************************************************/
void serialInit()
{ 
    try
    {
        ser.setPort("/dev/ttyUSB0");
        // ser.setPort("/dev/ttyUSB0");　// 这个端口号就是之前用cutecom看到的端口名称
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_INFO_STREAM("Failed to open port");
        // return -1;
    }
    ROS_INFO_STREAM("Succeed to open port");
}

/********************************************************
函数功能：将机器人的线速度和角速度分解成左右轮子速度，打包发送给下位机
入口参数：参数来自于robot.cpp--robot:deal
出口参数：
********************************************************/
void writeSpeed(double RobotV, double YawRate, unsigned char StopFlag)
{
    unsigned char buf[8] = {0};//buf缓冲区与stm32上的数据对应，9个十六进制
    int i, length = 0;

    //计算左右轮期望速度
    if (RobotV == 0 && YawRate != 0) //旋转
    {
        leftVelSet.l = (short)(YawRate * ROBOT_RADIUS);//基础占空比是80
        rightVelSet.r = (short)(-YawRate * ROBOT_RADIUS);
    }
    
    if (RobotV != 0 && YawRate == 0) //直线
    {
        leftVelSet.l = (short)(-RobotV); //下位机驱动器采用的是占空比调速
        rightVelSet.r = (short)(-RobotV);//基础占空比是100
    }
//既有线速度，又有角速度
    if (RobotV > 0 && YawRate < 0)//右转
    {
        leftVelSet.l = (short)((-1)*(RobotV-YawRate * 0.1));//(short)( YawRate/30 * ((YawRate*1125)/(-RobotV*4655)+1.2));//简化运算后，将角速度转化成线速度取其一半为加权值
        rightVelSet.r = (short)((-1)*(RobotV+YawRate * 0.12));
    }
    if (RobotV > 0 && YawRate > 0)//左转
    {
        leftVelSet.l = (short)((-1)*(RobotV-YawRate * 0.09));//(short)( YawRate/30 * ((YawRate*1125)/(-RobotV*4655)+1.2));//简化运算后，将角速度转化成线速度取其一半为加权值
        rightVelSet.r = (short)((-1)*(RobotV+YawRate * 0.09));//(short)( YawRate/30 * ((YawRate*1125)/(-RobotV*4655)-1.2));// (short)(RobotV-YawRate * ROBOT_RADIUS/2);
    }

    

    if (RobotV < 0 && YawRate > 0)//左倒转
    {
        leftVelSet.l = (short)((-1)*(RobotV+YawRate * 0.075));//(short)( YawRate/30 * ((YawRate*1125)/(-RobotV*4655)+1.2));//简化运算后，将角速度转化成线速度取其一半为加权值
        rightVelSet.r  = (short)((-1)*(RobotV-YawRate * 0.05));
    }

    if (RobotV < 0 && YawRate < 0)//右倒转
    {
        leftVelSet.l = (short)((-1)*(RobotV+YawRate * 0.075));//(short)( YawRate/30 * ((YawRate*1125)/(-RobotV*4655)+1.2));//简化运算后，将角速度转化成线速度取其一半为加权值
        rightVelSet.r = (short)((-1)*(RobotV-YawRate * 0.05));
    }

    if (RobotV == 0 && YawRate == 0)
    {
        leftVelSet.l = (short)(RobotV+YawRate);//无用的计算，结果是0
        rightVelSet.r = (short)(RobotV+YawRate);//无用的计算，结果是0
    }
  
    ROS_INFO("leftV: %d rightV: %d", leftVelSet.l,rightVelSet.r);
    
    buf[0] = header;
    
    buf[1] = StopFlag;
    ROS_INFO("StopFlag: %x", StopFlag);
    
    if(buf[1] == 0x01)
    {
            for (i = 0; i < 2; i++)
       {
            buf[i + 2] = rightVelSet.data2[1 - i]; // buf[2] buf[3]
            buf[i + 4] = leftVelSet.data1[1 - i];  // buf[4] buf[5]
       }
       
    }
    else
    {
        buf[1]=0x02;
    }
    /* 试验的数据 */
    // buf[2]=0x00;
    // buf[3]=0xc8;
    // buf[4]=0x00;
    // buf[5]=0xc8;
    /* 如果在控制中出问题，将buf[6]设为校验值，进行一个简单的加法求和校验 */
    buf[6] = (buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]) & 0xFF;
    buf[7] = ender;
    
    /* 下发数据指令 */    
    /* write的声明：write (const uint8_t *data, size_t size); */
    ser.write(buf,sizeof(buf));

     ROS_INFO(" %x %x %x %x %x %x %x",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6]);
}

