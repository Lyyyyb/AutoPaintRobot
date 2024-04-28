#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <can_msgs/Frame.h>

// Constants definitions
const double MAX_LINEAR_SPEED = 1.0;
const double MIN_LINEAR_SPEED = -1.0;
const double MIN_SPEED_THRESHOLD = 0.1;
const double WHEEL_DISTANCE = 0.5;
const int16_t MAX_SPEED_VALUE = 32767;
const uint8_t CAN_FRAME_DLC = 8;

ros::Publisher pub;

// Define a union for easy conversion between int16_t and two uint8_t
union SpeedData {
    int16_t speed;
    uint8_t bytes[2];
};

void publishCanFrame(uint32_t id, uint8_t data[]) {
    can_msgs::Frame frame;
    frame.id = id;
    frame.dlc = CAN_FRAME_DLC;
    for (int i = 0; i < CAN_FRAME_DLC; ++i) {
        frame.data[i] = data[i];
    }
    pub.publish(frame);
}

void prepareAndPublishCanMessage(double v_l, double v_r) {
    v_l = std::min(std::max(v_l, MIN_LINEAR_SPEED), MAX_LINEAR_SPEED);
    v_r = std::min(std::max(v_r, MIN_LINEAR_SPEED), MAX_LINEAR_SPEED);

    SpeedData left_speed;
    SpeedData right_speed;

    left_speed.speed = static_cast<int16_t>(v_l * MAX_SPEED_VALUE);
    right_speed.speed = static_cast<int16_t>(v_r * MAX_SPEED_VALUE);

    uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_speed.bytes[0], left_speed.bytes[1], 0x00, 0x00};
    uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_speed.bytes[0], right_speed.bytes[1], 0x00, 0x00};

    publishCanFrame(0x601, data_l); // Left wheel
    publishCanFrame(0x602, data_r); // Right wheel
}

void joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    ROS_INFO("Linear Vel: %f, Angular Vel: %f", msg->axes[1], msg->axes[3]);

    double linear_vel = msg->axes[1];
    double angular_vel = msg->axes[3];
    double v_l, v_r;

    if (fabs(linear_vel) > MIN_SPEED_THRESHOLD && fabs(angular_vel) < MIN_SPEED_THRESHOLD) {
        v_l = linear_vel;
        v_r = linear_vel;
    } else if (fabs(linear_vel) < MIN_SPEED_THRESHOLD && fabs(angular_vel) > MIN_SPEED_THRESHOLD) {
        v_l = -angular_vel * WHEEL_DISTANCE / 2;
        v_r = angular_vel * WHEEL_DISTANCE / 2;
    } else {
        double v = linear_vel;
        double omega = angular_vel;
        v_l = v - omega * WHEEL_DISTANCE / 2;
        v_r = v + omega * WHEEL_DISTANCE / 2;
    }

    prepareAndPublishCanMessage(v_l, v_r);
}


void canCallback(const can_msgs::Frame::ConstPtr &msg) {
    // Handle received CAN messages here
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "joy_to_cmd_vel_node");
    ros::NodeHandle nh;

    pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);

    ros::Subscriber sub_joy = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);

    ros::Subscriber sub_can = nh.subscribe<can_msgs::Frame>("received_messages", 10, canCallback);

    ros::spin();

    return 0;
}


