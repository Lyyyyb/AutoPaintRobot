#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <boost/bind.hpp>
#include <mutex>

// 变量声明
double wheel_distance;
double max_speed_value;
int can_frame_dlc;
int can_id_left_wheel;
int can_id_right_wheel;
int loop_rate;
ros::Publisher can_pub;
std::mutex mtx;  // 互斥锁，用于同步对共享变量的访问

// 定义一个联合体，用于int16_t和两个uint8_t之间的数据转换
union SpeedData {
    int16_t value;
    uint8_t bytes[2];
};

// 存储上一次发送的速度值，避免重复发送相同的速度
int16_t last_left_speed = 0;
int16_t last_right_speed = 0;

bool left_motor_initialized = false;
bool right_motor_initialized = false;

void publishCanFrame(int id, uint8_t data[], ros::Publisher& pub) {
    can_msgs::Frame frame;
    frame.id = id;
    frame.dlc = can_frame_dlc;
    for (int i = 0; i < can_frame_dlc; ++i) {
        frame.data[i] = data[i];
    }
    pub.publish(frame);
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher& pub) {
    double v = msg->linear.x;
    double omega = msg->angular.z;

    if (v == 0 && omega == 0) {
        SpeedData stop_speed;
        stop_speed.value = 0;
        uint8_t stop_data[] = {0x2B, 0x01, 0x20, 0x00, stop_speed.bytes[0], stop_speed.bytes[1], 0x00, 0x00};
        publishCanFrame(can_id_left_wheel, stop_data, pub);
        publishCanFrame(can_id_right_wheel, stop_data, pub);
        return;
    }

    double v_l = v - omega * wheel_distance / 2;
    double v_r = v + omega * wheel_distance / 2;

    int duty_cycle_left = static_cast<int>((v_l / max_speed_value) * 1000);
    int duty_cycle_right = static_cast<int>((v_r / max_speed_value) * 1000);
    duty_cycle_left = std::max(std::min(duty_cycle_left, 1000), -1000);
    duty_cycle_right = std::max(std::min(duty_cycle_right, 1000), -1000);

    ROS_INFO("Left wheel duty cycle: %d, Right wheel duty cycle: %d", duty_cycle_left, duty_cycle_right);

    SpeedData left_duty_cycle, right_duty_cycle;
    left_duty_cycle.value = duty_cycle_left;
    right_duty_cycle.value = duty_cycle_right;

    // 锁定互斥锁
    mtx.lock();
    last_left_speed = duty_cycle_left;
    last_right_speed = duty_cycle_right;
    mtx.unlock();

    uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_duty_cycle.bytes[0], left_duty_cycle.bytes[1], 0x00, 0x00};
    uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_duty_cycle.bytes[0], right_duty_cycle.bytes[1], 0x00, 0x00};

    publishCanFrame(can_id_left_wheel, data_l, pub);
    publishCanFrame(can_id_right_wheel, data_r, pub);
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
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, boost::bind(cmdVelCallback, _1, boost::ref(can_pub)));

    // 使用MultiThreadedSpinner允许多个回调并行执行
    ros::MultiThreadedSpinner spinner(4); // 使用4个线程
    spinner.spin(); // spin() 会不断调用 ros::spinOnce()

    ros::shutdown();
    return 0;
}
