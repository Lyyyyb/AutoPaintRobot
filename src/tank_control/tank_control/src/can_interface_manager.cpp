#include "tank_control/can_interface_manager.h"

// std::string can_device_;
extern ros::Publisher can_pub_;   
extern ros::Subscriber can_sub_; 

namespace CANInterfaceManager {
    void frameCallback(const can_msgs::Frame::ConstPtr& frame) {
        sendFrame(*frame);
    }

    void logFrame(const can_msgs::Frame& frame) {
        std::cout << "Sending CAN Frame - ID: " << std::hex << frame.id << std::dec
                  << ", Data: ";
        for (auto byte : frame.data) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
        }
        std::cout << std::endl;
    }

    void sendFrame(const can_msgs::Frame& frame) {
        if (!can_pub_) {
            std::cerr << "错误: CAN 发布器未初始化。" << std::endl;
            return;
        }

        logFrame(frame);

        try {
            can_pub_.publish(frame);
            // std::cout << "已成功发送 CAN 帧。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "发送 CAN 帧出现异常: " << e.what() << std::endl;
        }
    }
}
