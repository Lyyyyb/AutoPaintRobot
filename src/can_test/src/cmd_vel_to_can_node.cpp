// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <can_msgs/Frame.h>
// #include <cmath>
// #include <boost/bind.hpp>
// #include <queue>
// #include <mutex>

// // 变量声明
// double wheel_distance; // 轮距，单位为米
// double max_speed_value; // 最大速度值，用于速度的数值转换
// int can_frame_dlc; // CAN帧的数据长度
// int can_id_left_wheel; // 左轮CAN ID
// int can_id_right_wheel; // 右轮CAN ID
// int loop_rate; // 循环频率
// ros::Publisher can_pub; // 定义一个发布者，用于发送CAN消息
// std::mutex mtx;  // 定义全局互斥锁
// // 定义一个联合体，用于int16_t和两个uint8_t之间的数据转换
// union SpeedData {
//     int16_t value; // 存储速度值
//     uint8_t bytes[2]; // 存储速度值的字节形式
// };

// // 存储上一次发送的速度值，避免重复发送相同的速度
// int16_t last_left_speed = 0;
// int16_t last_right_speed = 0;

// // 电机初始化状态
// bool left_motor_initialized = false;
// bool right_motor_initialized = false;

// bool left_motor_ready = false;
// bool right_motor_ready = false;

// std::queue<geometry_msgs::Twist::ConstPtr> cmd_queue; // 命令队列，存储待发送的命令
// const size_t max_cmd_queue_size = 100; // 队列最大长度
// // 定义发布CAN帧的函数
// void publishCanFrame(int id, uint8_t data[], ros::Publisher& pub) {
//     can_msgs::Frame frame;
//     frame.id = id;
//     frame.dlc = can_frame_dlc;
//     for (int i = 0; i < can_frame_dlc; ++i) {
//         frame.data[i] = data[i];
//     }
//     // 使用传入的发布者对象进行发布
//     try {
//         pub.publish(frame);
//     } catch (const ros::Exception& ex) {
//         ROS_ERROR("Failed to publish CAN frame: %s", ex.what());
//         // 在这里可以执行其他错误处理操作，如记录错误日志或重新尝试发布
//     }
// }
// // 处理cmd_vel话题消息的回调函数
// void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher& pub) {
//     //mtx.lock();  // 锁定互斥锁

//     if (left_motor_ready && right_motor_ready) {
//         // 电机就绪，直接发送命令
//         double v = msg->linear.x; // 读取线速度
//         double omega = msg->angular.z; // 读取角速度

//         // 如果线速度和角速度都为零，则停止车辆
//         if (v == 0 && omega == 0) {
//             // 停止车辆，发送零速度命令
//             SpeedData stop_speed;
//             stop_speed.value = 0;

//             uint8_t stop_data[] = {0x2B, 0x01, 0x20, 0x00, stop_speed.bytes[0], stop_speed.bytes[1], 0x00, 0x00};

//             publishCanFrame(can_id_left_wheel, stop_data, pub); // 发布左轮CAN帧，传入发布者对象
//             publishCanFrame(can_id_right_wheel, stop_data, pub); // 发布右轮CAN帧，传入发布者对象

//             // 更新电机就绪状态为true
//             left_motor_ready = true;
//             right_motor_ready = true;

//             return; // 停止后不再继续执行后面的代码
//         }

//         double v_l = v - omega * wheel_distance / 2; // 计算左轮速度
//         double v_r = v + omega * wheel_distance / 2; // 计算右轮速度

//         // 转换速度到占空比，范围 -1000 到 1000
//         int duty_cycle_left = static_cast<int>((v_l / max_speed_value) * 1000);
//         int duty_cycle_right = static_cast<int>((v_r / max_speed_value) * 1000);
//         duty_cycle_left = std::max(std::min(duty_cycle_left, 1000), -1000);
//         duty_cycle_right = std::max(std::min(duty_cycle_right, 1000), -1000);

//         // 如果左右轮占空比没有变化，则不发送CAN帧
//         if (duty_cycle_left == last_left_speed && duty_cycle_right == last_right_speed) {
//             return;
//         }

//         SpeedData left_duty_cycle, right_duty_cycle;
//         left_duty_cycle.value = duty_cycle_left;
//         right_duty_cycle.value = duty_cycle_right;

//         // 更新上一次发送的占空比
//         last_left_speed = duty_cycle_left;
//         last_right_speed = duty_cycle_right;

//         // 构造左轮和右轮的CAN数据数组
//         uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_duty_cycle.bytes[0], left_duty_cycle.bytes[1], 0x00, 0x00};
//         uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_duty_cycle.bytes[0], right_duty_cycle.bytes[1], 0x00, 0x00};

//         publishCanFrame(can_id_left_wheel, data_l, pub); // 发布左轮CAN帧，传入发布者对象
//         publishCanFrame(can_id_right_wheel, data_r, pub); // 发布右轮CAN帧，传入发布者对象

//     } else {
//         // 电机未就绪，将命令存储在队列中
//         if (!left_motor_ready) {
//             ROS_WARN("Left motor not ready. Command stored in queue.");
//         }
//         if (!right_motor_ready) {
//             ROS_WARN("Right motor not ready. Command stored in queue.");
//         }
//         // 添加队列长度限制，丢弃最旧的命令
//         if (cmd_queue.size() < max_cmd_queue_size) {
//             cmd_queue.push(msg);
//         } else {
//             ROS_WARN("Command queue is full. Discarding oldest command.");
//             cmd_queue.pop();
//             cmd_queue.push(msg);
//         }
//     }

//     //mtx.unlock();  // 解锁互斥锁
// }

// void initMotors(ros::Publisher& pub) {
//     // 初始化电机函数
//     uint8_t init_motor_data[] = {0x2f, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
//     int max_attempts = 10; // 最大尝试次数
//     int attempts = 0; // 当前尝试次数

//     // 重置电机初始化状态和就绪状态
//     left_motor_ready = false;
//     right_motor_ready = false;

//     // 当电机未完全初始化且尝试次数未超过最大次数时循环
//     while (!(left_motor_initialized && right_motor_initialized) && attempts < max_attempts) {
//         // 发送初始化电机的CAN帧
//         publishCanFrame(can_id_left_wheel, init_motor_data, pub);
//         publishCanFrame(can_id_right_wheel, init_motor_data, pub);

//         ros::spinOnce(); // 处理回调，检查电机是否已响应初始化命令
//         ros::Duration(0.5).sleep(); // 等待电机响应
//         attempts++; // 尝试次数加一
//         ROS_INFO("Attempt %d to initialize motors", attempts);

//         // 检查电机是否已初始化
//         if (left_motor_initialized && right_motor_initialized) {
//             // 电机已成功初始化
//             ROS_INFO("Both motors initialized successfully.");
//             left_motor_ready = true;  // 设置电机就绪状态
//             right_motor_ready = true; // 设置电机就绪状态
//             break; // 成功初始化后退出循环
//         }
//     }

//     if (!left_motor_initialized || !right_motor_initialized) {
//         // 电机初始化失败
//         ROS_ERROR("Failed to initialize motors after %d attempts. Exiting program.", attempts);
//         ros::shutdown(); // 关闭ROS节点
//         exit(EXIT_FAILURE); // 退出程序并返回失败状态
//     }
// }


// // 处理CAN消息的回调函数
// void canCallBack(const can_msgs::Frame::ConstPtr& msg,ros::Publisher& pub) {
//     //mtx.lock();  // 锁定互斥锁

//     // 异步处理CAN消息，以减少主线程的阻塞时间
//     //ros::AsyncSpinner spinner(1);
//     //spinner.start();

//     switch (msg->id) {
//         case 0x581: // 左轮电机的CAN ID
//             // 处理左轮电机就绪状态
//             if (msg->data[0] == 0x60 && msg->data[1] == 0x00 && msg->data[2] == 0x20) {
//                 left_motor_ready = true;
//                 ROS_INFO("Left motor initialized and ready for new commands.");
//             } else {
//                 // 重置左轮电机就绪状态为false
//                 left_motor_ready = false;
//             }
//             break;
//         case 0x582: // 右轮电机的CAN ID
//             // 处理右轮电机就绪状态
//             if (msg->data[0] == 0x60 && msg->data[1] == 0x00 && msg->data[2] == 0x20) {
//                 right_motor_ready = true;
//                 ROS_INFO("Right motor initialized and ready for new commands.");
//             } else {
//                 // 重置右轮电机就绪状态为false
//                 right_motor_ready = false;
//             }
//             break;
//         default:
//             // 其他CAN ID的处理逻辑
//             break;
//     }

//     // 如果左右轮电机都准备好，且队列中有待发送的命令，则发送命令
//     if (left_motor_ready && right_motor_ready && !cmd_queue.empty()) {
//         // 从队列中获取命令并发送
//         geometry_msgs::Twist::ConstPtr next_cmd = cmd_queue.front();
//         cmd_queue.pop();
//         // 发送命令的逻辑...
//         // 提取命令中的线速度和角速度信息
//         double v = next_cmd->linear.x;
//         double omega = next_cmd->angular.z;

//         // 计算左右轮速度
//         double v_l = v - omega * wheel_distance / 2;
//         double v_r = v + omega * wheel_distance / 2;

//         // 转换速度到占空比，范围 -1000 到 1000
//         int duty_cycle_left = static_cast<int>((v_l / max_speed_value) * 1000);
//         int duty_cycle_right = static_cast<int>((v_r / max_speed_value) * 1000);
//         duty_cycle_left = std::max(std::min(duty_cycle_left, 1000), -1000);
//         duty_cycle_right = std::max(std::min(duty_cycle_right, 1000), -1000);

//         // 构造左轮和右轮的CAN数据数组
//         uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, static_cast<uint8_t>(duty_cycle_left & 0xFF), static_cast<uint8_t>((duty_cycle_left >> 8) & 0xFF), 0x00, 0x00};
//         uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, static_cast<uint8_t>(duty_cycle_right & 0xFF), static_cast<uint8_t>((duty_cycle_right >> 8) & 0xFF), 0x00, 0x00};

//         // 发布左轮和右轮的CAN帧
//         publishCanFrame(can_id_left_wheel, data_l, pub);
//         publishCanFrame(can_id_right_wheel, data_r, pub);
//     } 
//     //spinner.stop(); // 停止异步 Spinner
//     //mtx.unlock();  // 解锁互斥锁
// }
// int main(int argc, char **argv) {
//     // 初始化ROS节点
//     ros::init(argc, argv, "cmd_vel_to_can_node");
//     // 创建节点句柄
//     ros::NodeHandle nh;

//     // 设置参数
//     nh.param("wheel_distance", wheel_distance, 0.8); // 设置轮距，默认为0.8米
//     nh.param("max_speed_value", max_speed_value, 1.0); // 设置最大速度值，默认为1.0 m/s
//     nh.param("can_frame_dlc", can_frame_dlc, 8); // 设置CAN帧数据长度，默认为8
//     nh.param("can_id_left_wheel", can_id_left_wheel, 0x601); // 设置左轮CAN ID，默认为0x601
//     nh.param("can_id_right_wheel", can_id_right_wheel, 0x602); // 设置右轮CAN ID，默认为0x602
//     nh.param("loop_rate", loop_rate, 10); // 设置循环频率，默认为10Hz

//     can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 1000); // 创建CAN消息发布者对象
//     // 订阅/cmd_vel话题，接收机器人运动指令，设置回调函数为cmdVelCallback，队列长度为1000
//     ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, boost::bind(cmdVelCallback, _1, boost::ref(can_pub)));
//     // 订阅接收CAN消息的话题/received_messages，设置回调函数为canCallBack，队列长度为1000
//     ros::Subscriber sub = nh.subscribe<can_msgs::Frame>("/received_messages", 1000, boost::bind(canCallBack, _1, boost::ref(can_pub)));

//     // 初始化电机状态
//     initMotors(can_pub);

//     // 设置循环频率
//     ros::Rate rate(loop_rate);
//     while (ros::ok()) {
//         // 记录开始时间
//         ros::Time start = ros::Time::now();

//         // 处理所有回调函数
//         ros::spinOnce();

//         // 记录结束时间
//         ros::Time end = ros::Time::now();

//         // 计算实际执行频率
//         double actual_frequency = 1.0 / (end - start).toSec();

//         // 检查实际执行频率是否超出设定范围
//         if (actual_frequency > loop_rate) {
//             ROS_WARN("Actual loop frequency (%.2f Hz) exceeds desired loop rate (%d Hz)", actual_frequency, loop_rate);
//         }

//         // 休眠直到下一个周期开始
//         rate.sleep();
//     }

//     // 关闭ROS节点
//     ros::shutdown();
//     return 0; // 返回0表示正常退出
// }






// 初始化电机函数
// void initMotors(ros::Publisher& pub) {
//     uint8_t init_motor_data[] = {0x2f, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
//     int max_attempts = 10;
//     int attempts = 0;

//     while (!(left_motor_initialized && right_motor_initialized) && attempts < max_attempts) {
//         publishCanFrame(0x601, init_motor_data, pub);
//         publishCanFrame(0x602, init_motor_data, pub);
//         ros::spinOnce(); // 确保处理CAN帧的回调函数
//         ros::Duration(0.5).sleep();
//         attempts++;
//         ROS_INFO("Attempt %d to initialize motors", attempts);

//         if (left_motor_initialized && right_motor_initialized) {
//             ROS_INFO("Both motors initialized successfully.");
//             break; // 如果两个电机都已初始化，立即退出循环
//         }
//     }

//     if (!left_motor_initialized || !right_motor_initialized) {
//         ROS_ERROR("Failed to initialize motors after %d attempts. Continuing with limited functionality.", attempts);
//     }
// }
// CAN消息回调函数
// void canCallBack(const can_msgs::Frame::ConstPtr& msg) {
//     switch (msg->id) {
//         case 0x581:
//         case 0x582:
//             // 确保data[0]为0x60, data[1]为0x00, 和data[2]为0x20
//             if (msg->data[0] == 0x60 && msg->data[1] == 0x00 && msg->data[2] == 0x20) {
//                 bool isLeft = (msg->id == 0x581);
//                 if (isLeft) {
//                     left_motor_initialized = true;
//                     ROS_INFO("Left motor initialized successfully!");
//                 } else {
//                     right_motor_initialized = true;
//                     ROS_INFO("Right motor initialized successfully!");
//                 }
//             } else {
//                 ROS_WARN("Unexpected data in known frame 0x%X: %02X %02X %02X", msg->id, msg->data[0], msg->data[1], msg->data[2]);
//             }
//             break;

//         default:
//             ROS_WARN("Ignoring unexpected CAN frame with ID: 0x%X, data: %02X %02X %02X %02X %02X %02X %02X %02X",
//                      msg->id, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
//             break;
//     }
// }


// void canCallBack(const can_msgs::Frame::ConstPtr& msg) {
//     switch (msg->id) {
//         case 0x581: // 左轮电机的CAN ID
//         case 0x582: // 右轮电机的CAN ID
//             // 确保data[0]为0x60, data[1]为0x00, 和data[2]为0x20
//             if (msg->data[0] == 0x60 && msg->data[1] == 0x00 && msg->data[2] == 0x20) {
//                 if (msg->id == 0x581) {
//                     left_motor_ready = true; // 设置左轮电机就绪状态
//                     ROS_INFO("Left motor initialized and ready for new commands.");
//                 } else if (msg->id == 0x582) {
//                     right_motor_ready = true; // 设置右轮电机就绪状态
//                     ROS_INFO("Right motor initialized and ready for new commands.");
//                 }
//             } else {
//                 // 如果电机未就绪，重新发送上一条命令
//                 if (last_cmd_msg) {
//                     cmdVelCallback(last_cmd_msg, can_pub); // 重新发送上一条命令
//                     ROS_WARN("Motor with ID 0x%X not ready. Data: %02X %02X %02X. Resending command.", msg->id, msg->data[0], msg->data[1], msg->data[2]);
//                 } else {
//                     ROS_WARN("No previous command to resend.");
//                 }
//             }
//             break;

//         default:
//             // 处理未知的或意外的CAN ID
//             ROS_WARN("Ignoring unexpected CAN frame with ID: 0x%X, data: %02X %02X %02X %02X %02X %02X %02X %02X",
//                      msg->id, msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
//             break;
//     }
// }
// 处理cmd_vel话题消息的回调函数
// void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher& pub) {

//     if (!left_motor_ready || !right_motor_ready) {
//         ROS_WARN("Motors not ready. Skipping command.");
//         return;
//     }
//     last_cmd_msg = msg; // 保存当前命令以便重新发送
//     double v = msg->linear.x; // 读取线速度
//     double omega = msg->angular.z; // 读取角速度

//     // 如果线速度和角速度都为零，则停止车辆
//     if (v == 0 && omega == 0) {
//         // 停止车辆，发送零速度命令
//         SpeedData stop_speed;
//         stop_speed.value = 0;

//         uint8_t stop_data[] = {0x2B, 0x01, 0x20, 0x00, stop_speed.bytes[0], stop_speed.bytes[1], 0x00, 0x00};

//         publishCanFrame(can_id_left_wheel, stop_data, pub); // 发布左轮CAN帧，传入发布者对象
//         publishCanFrame(can_id_right_wheel, stop_data, pub); // 发布右轮CAN帧，传入发布者对象

//         return; // 停止后不再继续执行后面的代码
//     }

//     double v_l = v - omega * wheel_distance / 2; // 计算左轮速度
//     double v_r = v + omega * wheel_distance / 2; // 计算右轮速度

//     // 转换速度到占空比，范围 -1000 到 1000
//     int duty_cycle_left = static_cast<int>((v_l / max_speed_value) * 1000);
//     int duty_cycle_right = static_cast<int>((v_r / max_speed_value) * 1000);
//     duty_cycle_left = std::max(std::min(duty_cycle_left, 1000), -1000);
//     duty_cycle_right = std::max(std::min(duty_cycle_right, 1000), -1000);

//     // 日志输出当前占空比
//     ROS_INFO("Left wheel duty cycle: %d, Right wheel duty cycle: %d", duty_cycle_left, duty_cycle_right);

//     // 如果左右轮占空比没有变化，则不发送CAN帧
//     if (duty_cycle_left == last_left_speed && duty_cycle_right == last_right_speed) {
//         return;
//     }

//     SpeedData left_duty_cycle, right_duty_cycle;
//     left_duty_cycle.value = duty_cycle_left;
//     right_duty_cycle.value = duty_cycle_right;

//     // 更新上一次发送的占空比
//     last_left_speed = duty_cycle_left;
//     last_right_speed = duty_cycle_right;

//     // 构造左轮和右轮的CAN数据数组
//     uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_duty_cycle.bytes[0], left_duty_cycle.bytes[1], 0x00, 0x00};
//     uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_duty_cycle.bytes[0], right_duty_cycle.bytes[1], 0x00, 0x00};

//     publishCanFrame(can_id_left_wheel, data_l, pub); // 发布左轮CAN帧，传入发布者对象
//     publishCanFrame(can_id_right_wheel, data_r, pub); // 发布右轮CAN帧，传入发布者对象

//     // Reset ready status after sending commands
//     left_motor_ready = false;
//     right_motor_ready = false;
// }

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <boost/bind.hpp>

// 全局变量声明
double wheel_distance; // 轮间距离，用于计算差分驱动的左右轮速度
double max_speed_value; // 最大速度值，用于归一化速度计算
int can_frame_dlc; // CAN帧的数据长度代码
int can_id_left_wheel; // 左轮电机的CAN ID
int can_id_right_wheel; // 右轮电机的CAN ID
int loop_rate; // 节点循环频率
ros::Publisher can_pub; // 发布者，用于发送CAN消息
const double MAX_ANGULAR_VELOCITY = 1.0; // 最大角速度值，单位弧度/秒
const double MAX_LINEAR_SPEED = 1.0; // 最大线速度值，单位米/秒

// 速度数据的联合体，用于方便地转换速度值与字节流
union SpeedData {
    int16_t value;
    uint8_t bytes[2];
};

// 上一次发送的速度值，用于减少通信量
int last_left_speed = 0;
int last_right_speed = 0;

// 发布CAN帧的函数
void publishCanFrame(int id, uint8_t data[], ros::Publisher& pub) {
    can_msgs::Frame frame;
    frame.id = id; // 设置CAN ID
    frame.dlc = can_frame_dlc; // 设置数据长度代码
    for (int i = 0; i < can_frame_dlc; ++i) {
        frame.data[i] = data[i]; // 填充数据
    }
    pub.publish(frame); // 发布CAN帧
}

// 初始化电机的函数
void initMotors(ros::Publisher& pub) {
    uint8_t init_data[] = {0x2F, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00}; // 初始化数据
    int max_attempts = 5; // 最大尝试次数
    int attempts = 0;
    
    while (attempts < max_attempts) {
        publishCanFrame(can_id_left_wheel, init_data, pub); // 初始化左轮
        publishCanFrame(can_id_right_wheel, init_data, pub); // 初始化右轮
        ros::Duration(0.5).sleep(); // 间隔0.5秒
        attempts++;
        ROS_INFO("Motor initialization attempt %d", attempts);
    }
    ROS_INFO("Initialization commands sent %d times to motors.", max_attempts);
}

// cmd_vel话题的回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher& pub) {
    static double prev_v = 0.0; // 上次的线速度
    static double prev_omega = 0.0; // 上次的角速度
    double v = msg->linear.x; // 当前线速度
    double omega = msg->angular.z; // 当前角速度
    
    // 检查速度是否有变化
    if (v == prev_v && omega == prev_omega) {
        return; // 如果速度未变，不发送新的CAN消息
    }

    prev_v = v;
    prev_omega = omega;
    
    double v_l = v - omega * wheel_distance / 2; // 计算左轮速度
    double v_r = v + omega * wheel_distance / 2; // 计算右轮速度

    int duty_cycle_left = static_cast<int>((v_l / max_speed_value) * 1000); // 左轮速度占空比
    int duty_cycle_right = static_cast<int>((v_r / max_speed_value) * 1000); // 右轮速度占空比

    // 限制占空比范围为[-1000, 1000]
    duty_cycle_left = std::max(std::min(duty_cycle_left, 1000), -1000);
    duty_cycle_right = std::max(std::min(duty_cycle_right, 1000), -1000);

    SpeedData left_duty_cycle, right_duty_cycle;
    left_duty_cycle.value = duty_cycle_left;
    right_duty_cycle.value = duty_cycle_right;

    // 准备发送的数据
    uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_duty_cycle.bytes[0], left_duty_cycle.bytes[1], 0x00, 0x00};
    uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_duty_cycle.bytes[0], right_duty_cycle.bytes[1], 0x00, 0x00};

    publishCanFrame(can_id_left_wheel, data_l, pub); // 发送左轮数据
    publishCanFrame(can_id_right_wheel, data_r, pub); // 发送右轮数据

    ROS_INFO("Left wheel duty cycle: %d, Right wheel duty cycle: %d", duty_cycle_left, duty_cycle_right);
}

// 主函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_to_can_node"); // 初始化节点
    ros::NodeHandle nh; // 节点句柄

    // 从参数服务器获取参数
    nh.param("wheel_distance", wheel_distance, 1.0);
    nh.param("max_speed_value", max_speed_value, 1.0);
    nh.param("can_frame_dlc", can_frame_dlc, 8);
    nh.param("can_id_left_wheel", can_id_left_wheel, 0x601);
    nh.param("can_id_right_wheel", can_id_right_wheel, 0x602);
    nh.param("loop_rate", loop_rate, 200);

    // 初始化发布者和订阅者
    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, boost::bind(cmdVelCallback, _1, boost::ref(can_pub)));

    // 初始化电机
    initMotors(can_pub);

    ros::Rate rate(loop_rate); // 设置循环频率
    while (ros::ok()) {
        ros::spinOnce(); // 处理回调函数
        rate.sleep(); // 控制循环频率
    }

    ros::shutdown(); // 关闭ROS
    return 0;
}

// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <can_msgs/Frame.h>
// #include <cmath>
// #include <boost/bind.hpp>

// // 定义一个类来封装控制逻辑
// class CmdVelToCAN {
// private:
//     ros::NodeHandle nh;  // ROS节点句柄
//     ros::Publisher can_pub;  // CAN消息的发布者
//     ros::Subscriber cmd_vel_sub;  // cmd_vel消息的订阅者
//     double wheel_distance;  // 轮间距离，用于计算差分驱动的左右轮速度
//     double max_speed_value;  // 可接受的最大速度值，用于规范化速度
//     int can_frame_dlc;  // CAN帧的数据长度代码
//     int can_id_left_wheel;  // 左轮电机的CAN ID
//     int can_id_right_wheel;  // 右轮电机的CAN ID
//     double prev_v = 0.0;  // 存储上一次的线速度，用于检测变化
//     double prev_omega = 0.0;  // 存储上一次的角速度，用于检测变化

// public:
//     CmdVelToCAN() : nh("~"), wheel_distance(1.0), max_speed_value(1.0), can_frame_dlc(8), prev_v(0.0), prev_omega(0.0) {
//         if (!nh.getParam("wheel_distance", wheel_distance) ||
//             !nh.getParam("max_speed_value", max_speed_value) ||
//             !nh.getParam("can_frame_dlc", can_frame_dlc) ||
//             !nh.getParam("can_id_left_wheel", can_id_left_wheel) ||
//             !nh.getParam("can_id_right_wheel", can_id_right_wheel)) {
//             throw std::runtime_error("Failed to get parameters");
//         }
//         can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);
//         cmd_vel_sub = nh.subscribe("cmd_vel", 1000, &CmdVelToCAN::cmdVelCallback, this);
//         initMotors();
//     }
//     void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
//         double v = msg->linear.x;  // 获取当前线速度
//         double omega = msg->angular.z;  // 获取当前角速度

//         // 如果速度和角速度未变化，不更新
//         if (v == prev_v && omega == prev_omega) return;

//         prev_v = v;
//         prev_omega = omega;

//         publishWheelCommands(v, omega);
//     }

// private:
//     void initMotors() {
//         uint8_t init_data[] = {0x2F, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
//         for (int i = 0; i < 5; ++i) {
//             publishCanFrame(can_id_left_wheel, init_data);
//             publishCanFrame(can_id_right_wheel, init_data);
//             ros::Duration(0.5).sleep();
//             ROS_INFO("Motor initialization attempt %d", i + 1);
//         }
//         ROS_INFO("Initialization commands sent 5 times to motors.");
//     }

//     void publishWheelCommands(double v, double omega) {
//         double v_l = v - omega * wheel_distance / 2;  // 计算左轮速度
//         double v_r = v + omega * wheel_distance / 2;  // 计算右轮速度

//         // 计算左右轮的占空比
//         int duty_cycle_left = std::max(std::min(static_cast<int>((v_l / max_speed_value) * 1000), 1000), -1000);
//         int duty_cycle_right = std::max(std::min(static_cast<int>((v_r / max_speed_value) * 1000), 1000), -1000);

//         uint8_t data_l[8] = {0x2B, 0x01, 0x20, 0x00, static_cast<uint8_t>(duty_cycle_left & 0xFF), static_cast<uint8_t>((duty_cycle_left >> 8) & 0xFF), 0x00, 0x00};
//         uint8_t data_r[8] = {0x2B, 0x01, 0x20, 0x00, static_cast<uint8_t>(duty_cycle_right & 0xFF), static_cast<uint8_t>((duty_cycle_right >> 8) & 0xFF), 0x00, 0x00};

//         publishCanFrame(can_id_left_wheel, data_l);  // 发送左轮数据
//         publishCanFrame(can_id_right_wheel, data_r);  // 发送右轮数据
//     }

//     void publishCanFrame(int id, uint8_t data[]) {
//         can_msgs::Frame frame;
//         frame.id = id;  // 设置CAN ID
//         frame.dlc = can_frame_dlc;  // 设置数据长度代码
//         for (int i = 0; i < can_frame_dlc; ++i) {
//             frame.data[i] = data[i];  // 填充数据
//         }
//         can_pub.publish(frame);  // 发布CAN帧
//     }
// };

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "cmd_vel_to_can_node");
//     try {
//         CmdVelToCAN cmd_vel_to_can;
//         ros::spin();
//     } catch (const std::exception& e) {
//         ROS_FATAL_STREAM("Initialization failed: " << e.what());
//         return 1;
//     }
//     return 0;
// }



    // if (v == 0 && omega == 0) {
    //     SpeedData stop_speed;
    //     stop_speed.value = 0;
    //     uint8_t stop_data[] = {0x2B, 0x01, 0x20, 0x00, stop_speed.bytes[0], stop_speed.bytes[1], 0x00, 0x00};
    //     publishCanFrame(can_id_left_wheel, stop_data, pub);
    //     publishCanFrame(can_id_right_wheel, stop_data, pub);
    //     ROS_INFO("Stopping both wheels");
    //     return;
    // }