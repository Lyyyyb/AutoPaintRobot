#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Bool.h>
#include <boost/array.hpp>

ros::Publisher left_motor_ready_pub;
ros::Publisher right_motor_ready_pub;

void canCallBack(const can_msgs::Frame::ConstPtr& msg) {
    std_msgs::Bool ready_msg;
    ready_msg.data = (msg->data[0] == 0x60 && msg->data[1] == 0x00 && msg->data[2] == 0x20);

    if (msg->id == 0x581) { // 左轮电机CAN ID
        left_motor_ready_pub.publish(ready_msg);
        if (ready_msg.data) {
            ROS_INFO("Left motor ready for new commands.");
        } else {
            ROS_WARN("Left motor not ready.");
        }
    } else if (msg->id == 0x582) { // 右轮电机CAN ID
        right_motor_ready_pub.publish(ready_msg);
        if (ready_msg.data) {
            ROS_INFO("Right motor ready for new commands.");
        } else {
            ROS_WARN("Right motor not ready.");
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_message_processor");
    ros::NodeHandle nh;

    left_motor_ready_pub = nh.advertise<std_msgs::Bool>("left_motor_ready", 10);
    right_motor_ready_pub = nh.advertise<std_msgs::Bool>("right_motor_ready", 10);
    ros::Subscriber can_sub = nh.subscribe("/received_messages", 1000, canCallBack);

    ros::spin();
    return 0;
}
