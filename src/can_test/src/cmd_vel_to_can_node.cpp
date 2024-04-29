#include <ros/ros.h> // 导入ROS相关的头文件
#include <geometry_msgs/Twist.h> // 机器人运动控制消息头文件
#include <can_msgs/Frame.h> // CAN总线消息头文件
#include <cmath> // 导入数学计算相关头文件

// 变量声明
double wheel_distance; // 轮距，单位为米
int max_speed_value; // 最大速度值，用于速度的数值转换
int can_frame_dlc; // CAN帧的数据长度
int can_id_left_wheel; // 左轮CAN ID
int can_id_right_wheel; // 右轮CAN ID
int loop_rate; // 循环频率

ros::Publisher can_pub; // 定义一个发布者，用于发送CAN消息

// 定义一个联合体，用于int16_t和两个uint8_t之间的数据转换
union SpeedData {
    int16_t speed; // 存储速度值
    uint8_t bytes[2]; // 存储速度值的字节形式
};

// 存储上一次发送的速度值，避免重复发送相同的速度
int16_t last_left_speed = 0;
int16_t last_right_speed = 0;

// 电机初始化状态
bool left_motor_initialized = false;
bool right_motor_initialized = false;
// 定义发布CAN帧的函数
// 修改can发布检查
// 修改发布CAN帧的函数，加入发布者参数
void publishCanFrame(int id, uint8_t data[], ros::Publisher& pub) {
    can_msgs::Frame frame;
    frame.id = id;
    frame.dlc = can_frame_dlc;
    for (int i = 0; i < can_frame_dlc; ++i) {
        frame.data[i] = data[i];
    }
    // 使用传入的发布者对象进行发布
    pub.publish(frame);
}
// 处理cmd_vel话题消息的回调函数
void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg, ros::Publisher& pub) {
    double v = msg->linear.x; // 读取线速度
    double omega = msg->angular.z; // 读取角速度
    double v_l = v - omega * wheel_distance / 2; // 计算左轮速度
    double v_r = v + omega * wheel_distance / 2; // 计算右轮速度

    SpeedData left_speed; // 左轮速度数据
    SpeedData right_speed; // 右轮速度数据

    // 速度缩放并转换为int16_t类型，同时处理溢出
    left_speed.speed = std::max(std::min(static_cast<int>(v_l * max_speed_value), 32767), -32768);
    right_speed.speed = std::max(std::min(static_cast<int>(v_r * max_speed_value), 32767), -32768);

    // 日志输出当前速度
    ROS_INFO("Left wheel speed: %d, Right wheel speed: %d", left_speed.speed, right_speed.speed);

    // 如果左右轮速度没有变化，则不发送CAN帧
    if (left_speed.speed == last_left_speed && right_speed.speed == last_right_speed) {
        return;
    }

    // 更新上一次发送的速度值
    last_left_speed = left_speed.speed;
    last_right_speed = right_speed.speed;

    // 构造左轮和右轮的CAN数据数组
    uint8_t data_l[] = {0x2B, 0x01, 0x20, 0x00, left_speed.bytes[0], left_speed.bytes[1], 0x00, 0x00};
    uint8_t data_r[] = {0x2B, 0x01, 0x20, 0x00, right_speed.bytes[0], right_speed.bytes[1], 0x00, 0x00};

    publishCanFrame(can_id_left_wheel, data_l, pub); // 发布左轮CAN帧，传入发布者对象
    publishCanFrame(can_id_right_wheel, data_r, pub); // 发布右轮CAN帧，传入发布者对象
}

// 初始化电机函数
void initMotors(ros::Publisher& pub) {
    uint8_t init_motor_data[] = {0x2f, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
    int max_attempts = 10; 
    int attempts = 0;

    while (!(left_motor_initialized && right_motor_initialized) && attempts < max_attempts) {
        try {
            publishCanFrame(0x601, init_motor_data, pub);
            publishCanFrame(0x602, init_motor_data, pub);
            ros::Duration(0.5).sleep();  
            attempts++;
            ROS_INFO("Attempt %d to initialize motors", attempts);
        } catch (const std::exception& e) {
            ROS_ERROR("Exception caught during motor initialization: %s", e.what());
            ros::Duration(1.0).sleep(); // Wait before retrying
        }
    }

    if (left_motor_initialized && right_motor_initialized) {
        ROS_INFO("Both motors initialized successfully after %d attempts.", attempts);
    } else {
        ROS_ERROR("Failed to initialize motors after %d attempts. Continuing with limited functionality.");
    }
}


// CAN消息回调函数
void canCallBack(const can_msgs::Frame::ConstPtr& msg) {
    // 对于预期的CAN ID，进行正常处理
    switch (msg->id) {
        case 0x581:
        case 0x582:
            if (msg->data[0] == 0x60) {
                // 标准处理流程
                bool isLeft = msg->id == 0x581;
                if (isLeft && !left_motor_initialized) {
                    left_motor_initialized = true;
                    ROS_INFO("Left motor initialized successfully!");
                } else if (!right_motor_initialized) {
                    right_motor_initialized = true;
                    ROS_INFO("Right motor initialized successfully!");
                }
            } else {
                ROS_WARN("Unexpected data in known frame 0x%X: %02X", msg->id, msg->data[0]);
                if (!left_motor_initialized || !right_motor_initialized) {
                    initMotors(can_pub); // 尝试重新初始化
                }
            }
            break;

        default:
            // 对未知ID，记录警告而不是错误
            ROS_WARN("Ignoring unexpected CAN frame with ID: 0x%X, data: %02X", msg->id, msg->data[0]);
            break;
    }
}




int main(int argc, char **argv) {
    ros::init(argc, argv, "cmd_vel_to_can_node");
    ros::NodeHandle nh;

    // 获取参数
    nh.param("wheel_distance", wheel_distance, 0.5);
    nh.param("max_speed_value", max_speed_value, 3276);
    nh.param("can_frame_dlc", can_frame_dlc, 8);
    nh.param("can_id_left_wheel", can_id_left_wheel, 0x601);
    nh.param("can_id_right_wheel", can_id_right_wheel, 0x602);
    nh.param("loop_rate", loop_rate, 10);


    can_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 100, boost::bind(cmdVelCallback, _1, boost::ref(can_pub)));
    ros::Subscriber sub = nh.subscribe("/received_messages", 100, canCallBack);

    // 初始化电机
    initMotors(can_pub);

    ros::Rate rate(loop_rate);
    // 主循环中加入错误处理
    while (ros::ok()) {
        try {
            ros::spinOnce();
            rate.sleep();
        } catch (const std::exception& e) {
            ROS_ERROR("Exception caught: %s", e.what());
            // 可以选择重新初始化连接或其他恢复策略
        }
    }

    ros::shutdown();
    return 0;
}