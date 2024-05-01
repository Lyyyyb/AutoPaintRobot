#ifndef MY_ROBOT_CONTROLLER_H
#define MY_ROBOT_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>  // 用于订阅速度命令
#include "my_robot_hardware.h"   // 包含硬件接口
#include <controller_manager/controller_manager.h>  // 控制器管理器
#include "can_interface_manager.h"  // 包含 CAN 接口

extern float wheel_separation_;  // 轮子间距
extern float wheel_radius_;   // 轮子半径
extern int16_t left_wheel_velocity_;  // 左轮速度
extern int16_t right_wheel_velocity_;   // 右轮速度

ros::Publisher can_pub_;  // CAN 发布器
ros::Subscriber can_sub_; // CAN 订阅器
ros::Subscriber cmd_vel_sub_;  // 速度命令订阅器

#endif // MY_ROBOT_CONTROLLER_H
