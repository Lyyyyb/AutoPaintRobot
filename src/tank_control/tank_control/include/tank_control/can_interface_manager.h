#pragma once

#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <socketcan_interface/socketcan.h>

namespace CANInterfaceManager {
    void canMsgHandle(const can_msgs::Frame& frame);
    void sendFrame(const can_msgs::Frame& frame);
    void frameCallback(const can_msgs::Frame::ConstPtr& frame);
}

#endif // CAN_INTERFACE_H