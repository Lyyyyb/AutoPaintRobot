#include "tank_control/can_interface_manager.h"

// std::string can_device_;
extern ros::Publisher can_pub_;   
extern ros::Subscriber can_sub_; 

namespace CANInterfaceManager {
    void frameCallback(const can_msgs::Frame::ConstPtr& frame) {
        static int times = 0;
        std::cout << times++ << " ID: " << std::hex << std::setw(3) << std::setfill('0') << frame->id << " data: ";
        for (int i = 0; i < 8; ++i) {
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame->data[i]) << " ";
        }
        std::cout << std::endl;
        canMsgHandle(*frame);
    }

    void canMsgHandle(const can_msgs::Frame& frame) {
        sendFrame(frame);
    }

    void sendFrame(const can_msgs::Frame& frame) {
        if (!can_pub_) {
            std::cerr << "错误: CAN 发布器未初始化。" << std::endl;
            return;
        }

        // if (can_pub_.getNumSubscribers() < 1) {
        //     std::cerr << "警告: 还没有订阅者接收 CAN 帧。" << std::endl;
        // }

        try {
            can_pub_.publish(frame);
            // std::cout << "已成功发送 CAN 帧。" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "发送 CAN 帧出现异常: " << e.what() << std::endl;
        }
    }
}
