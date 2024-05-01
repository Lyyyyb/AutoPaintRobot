#ifndef MY_ROBOT_HARDWARE_H
#define MY_ROBOT_HARDWARE_H

#include "can_interface_manager.h"
#include "datatype_converter.h"
#include <ros/ros.h>
#include <string>
#include <stdexcept>
#include <memory>

namespace MyRobotHardware {
    // void read(const ros::Time& time, const ros::Duration& period);
    // void write(const ros::Time& time, const ros::Duration& period);

    void initMotor();
    void setMotorSpeed(int16_t left_speed, int16_t right_speed);
    void getMotorSpeed();
    void requestMotorStatus();

    template<typename T>
    void setMotorParameter(uint8_t canID, uint8_t Cmd, uint16_t Index, T Value);

    void getMotorParameter(uint16_t Index, uint8_t canID);
    // void handleCANMessage(const can_msgs::Frame::ConstPtr& msg);
    // 辅助函数：将4字节数组复制到wheel_frame.data[4]
    void setDataInFrame(can_msgs::Frame* frame, const uint8_t* bytes);
}

// Private defines
#define BASE_ADDRESS 0x4000
#define Send_Func_CAN_ID 0x600
#define Receive_Func_CAN_ID 0x580

// Configuration parameter storage register
#define Save_Motor_Param 0x0160
#define Save_ClosedLoop_Speed_PID_Param 0x01C0
#define Save_CAN_Param 0x01E0

// Index - Object Action Comparison Table
#define Control_Mode 0x2000 	// Read-write motor control type
#define Motion_Control 0x2001	// Send control cmd
#define Motor_Speed_Unit_ADDRESS 0x200A    // Read-write motor speed unit
#define Motor_Speed_Request 	0x210A	// Feedback the running speed of the motor to the host
#define Motor_Stall_Feedback	0x2111	// Get motor stall status
#define Error_Feedback 0x2112 	// Get motor fault status
#define CanID_ADDRESS 0x2201	// Read and write current motor CAN_ID
#define Motor_Frequency 0x2102 // Read motor commutation frequency
#define Motor_RPM 0x210A	// Read motor speed

// Read and write function comparison table
#define Get_Single_Param 0x40
#define Set_Single_Param 0x2F
#define Set_Two_Param 0x2B
#define Set_Three_Param 0x27
#define Set_Four_Param 0x23

// data effective number of bytes comparison table
#define Single_Byte_Valid 0x4F
#define Two_Byte_Valid	0x4B
#define Three_Byte_Valid	0x47
#define Four_Byte_Valid 0x43
#define All_Byte_Invalid	0x60	// transmission success
#define All_Byte_Valid 0x80	// transmission failure

//  real-time status register
#define Motor_Frequency_ADDRESS 0x0022
#define Motor_Stall_State_ADDRESS 0x0032
#define Error_State_ADDRESS 0x0033
#define Motor_RPM_ADDRESS 0x0034

#define RPM_Unit 0x01
#define Hz_Unit 0x00

#define PWM_Mode 0x00
#define Speed_Mode 0x01
#define Torque_Mode 0x02
#define Position_Mode 0x03
#define Normal_Stop 0x10
#define Emergency_Stop 0x11
#define Free_Stop 0x12

#endif
