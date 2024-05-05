#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <boost/bind.hpp>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>

// 变量声明
double wheel_distance;
double max_speed_value;
int can_frame_dlc;
int can_id_left_wheel;
int can_id_right_wheel;
int loop_rate;
ros::Publisher can_pub;

bool left_motor_ready = false; // 左轮电机就绪状态
bool right_motor_ready = false; // 右轮电机就绪状态
int last_left_speed = 0; // 上次发送的左轮速度
int last_right_speed = 0; // 上次发送的右轮速度

std::mutex mtx;  // 全局互斥锁
std::mutex queue_mutex; // 队列操作的互斥锁
std::condition_variable queue_cond; // 用于队列的条件变量
std::queue<geometry_msgs::Twist::ConstPtr> cmd_queue; // 消息队列
bool exit_thread = false; // 用于通知线程退出的标志

union SpeedData {
    int16_t value;
    uint8_t bytes[2];
};

void publishCanFrame(int id, uint8_t data[], ros::Publisher& pub) {
    can_msgs::Frame frame;
    frame.id = id;
    frame.dlc = can_frame_dlc;
    for (int i = 0; i < can_frame_dlc; ++i) {
        frame.data[i] = data[i];
    }
    pub.publish(frame);
}

void canCallBack(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher& pub) {
    if (left_motor_ready && right_motor_ready) {
        double v = msg->linear.x;  // 线速度
        double omega = msg->angular.z;  // 角速度

        // 当线速度和角速度都为零时，发送停止指令
        if (v == 0 && omega == 0) {
            SpeedData stop_speed;
            stop_speed.value = 0;
            uint8_t stop_data[] = {0x2B, 0x01, 0x20, 0x00, stop_speed.bytes[0], stop_speed.bytes[1], 0x00, 0x00};
            publishCanFrame(can_id_left_wheel, stop_data, pub);
            publishCanFrame(can_id_right_wheel, stop_data, pub);
            return;
        }

        // 计算左右轮的速度
        double v_l = v - omega * wheel_distance / 2;
        double v_r = v + omega * wheel_distance / 2;

        // 转换速度为占空比，范围-1000到1000
        int duty_cycle_left = static_cast<int>((v_l / max_speed_value) * 1000);
        int duty_cycle_right = static_cast<int>((v_r / max_speed_value) * 1000);
        duty_cycle_left = std::max(std::min(duty_cycle_left, 1000), -1000);
        duty_cycle_right = std::max(std::min(duty_cycle_right, 1000), -1000);

        // 只在占空比有变化时发送CAN帧
        if (duty_cycle_left != last_left_speed || duty_cycle_right != last_right_speed) {
            SpeedData left_duty_cycle, right_duty_cycle;
            left_duty_cycle.value = duty_cycle_left;
            right_duty_cycle.value = duty_cycle_right;

            uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_duty_cycle.bytes[0], left_duty_cycle.bytes[1], 0x00, 0x00};
            uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_duty_cycle.bytes[0], right_duty_cycle.bytes[1], 0x00, 0x00};

            publishCanFrame(can_id_left_wheel, data_l, pub);
            publishCanFrame(can_id_right_wheel, data_r, pub);

            // 更新上一次的占空比
            last_left_speed = duty_cycle_left;
            last_right_speed = duty_cycle_right;
        }
    } else {
        // 如果电机不就绪，这里可以添加日志或处理逻辑
        ROS_WARN("Motors are not ready. Skipping command.");
    }
}


void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(queue_mutex);
    if (cmd_queue.size() < 100) { // 限制队列大小
        cmd_queue.push(msg);
        queue_cond.notify_one();
    }
}

void processQueue() {
    std::unique_lock<std::mutex> lock(queue_mutex);
    while (!exit_thread || !cmd_queue.empty()) {
        if (cmd_queue.empty()) {
            queue_cond.wait(lock);
        } else {
            auto msg = cmd_queue.front();
            cmd_queue.pop();
            lock.unlock();
            canCallBack(msg, can_pub);
            lock.lock();
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_to_can_node");
    ros::NodeHandle nh;

    nh.param("wheel_distance", wheel_distance, 0.8);
    nh.param("max_speed_value", max_speed_value, 1.0);
    nh.param("can_frame_dlc", can_frame_dlc, 8);
    nh.param("can_id_left_wheel", can_id_left_wheel, 0x601);
    nh.param("can_id_right_wheel", can_id_right_wheel, 0x602);
    nh.param("loop_rate", loop_rate, 10);

    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, cmdVelCallback);

    std::thread processing_thread(processQueue);

    ros::spin();

    exit_thread = true;
    queue_cond.notify_all();
    processing_thread.join();

    return 0;
}
