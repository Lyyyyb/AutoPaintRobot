#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <vector>
#include <boost/bind.hpp>

bool left_motor_ready = false;
bool right_motor_ready = false;

double wheel_distance, max_speed_value;
int can_frame_dlc, can_id_left_wheel, can_id_right_wheel, loop_rate;
ros::Publisher can_pub;

void leftMotorReadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    left_motor_ready = msg->data;
}

void rightMotorReadyCallback(const std_msgs::Bool::ConstPtr& msg) {
    right_motor_ready = msg->data;
}

void publishCanFrame(int id, const std::vector<uint8_t>& data) {
    can_msgs::Frame frame;
    frame.id = id;
    frame.dlc = data.size();
    for (size_t i = 0; i < data.size(); ++i) {
        frame.data[i] = data[i];
    }
    can_pub.publish(frame);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!left_motor_ready || !right_motor_ready) {
        ROS_WARN("One or both motors not ready. Skipping command.");
        return;
    }

    double v = msg->linear.x;
    double omega = msg->angular.z;

    double v_l = v - omega * wheel_distance / 2;
    double v_r = v + omega * wheel_distance / 2;

    std::vector<uint8_t> data_l = {0x2B, 0x01, 0x20, 0x00, static_cast<uint8_t>((int(v_l / max_speed_value) * 1000) & 0xFF), static_cast<uint8_t>((int(v_l / max_speed_value) * 1000) >> 8), 0x00, 0x00};
    std::vector<uint8_t> data_r = {0x2B, 0x01, 0x20, 0x00, static_cast<uint8_t>((int(v_r / max_speed_value) * 1000) & 0xFF), static_cast<uint8_t>((int(v_r / max_speed_value) * 1000) >> 8), 0x00, 0x00};

    publishCanFrame(can_id_left_wheel, data_l);
    publishCanFrame(can_id_right_wheel, data_r);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_to_can_node");
    ros::NodeHandle nh;

    nh.param("wheel_distance", wheel_distance, 0.8);
    nh.param("max_speed_value", max_speed_value, 1.0);
    nh.param("can_frame_dlc", can_frame_dlc, 8);
    nh.param("can_id_left_wheel", can_id_left_wheel, 0x601);
    nh.param("can_id_right_wheel", can_id_right_wheel, 0x602);
    nh.param("loop_rate", loop_rate, 10);

    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);
    ros::Subscriber left_ready_sub = nh.subscribe<std_msgs::Bool>("left_motor_ready", 10, leftMotorReadyCallback);
    ros::Subscriber right_ready_sub = nh.subscribe<std_msgs::Bool>("right_motor_ready", 10, rightMotorReadyCallback);
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmdVelCallback);

    ros::Rate rate(loop_rate);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
