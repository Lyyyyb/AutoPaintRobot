#include "tank_control/my_robot_hardware.h"

extern float left_wheel_velocity_;
extern float right_wheel_velocity_;
uint8_t bytes[4] = { 0 };

namespace MyRobotHardware { 
    // void handleCANMessage(const can_msgs::Frame::ConstPtr& msg) {
    //     if (msg->id == 0x06) { // Left wheel status feedback
    //         left_wheel_velocity_ = static_cast<float>(msg->data[1]);
    //     } else if (msg->id == 0x07) { // Right wheel status feedback
    //         right_wheel_velocity_ = static_cast<float>(msg->data[1]);
    //     } else {
    //         ROS_WARN("Received unknown CAN message with ID: %x", msg->id);
    //     }
    // }

    void initMotor() {        
        setMotorParameter(0x01, Set_Single_Param, Motor_Speed_Unit_ADDRESS, RPM_Unit);
        setMotorParameter(0x02, Set_Single_Param, Motor_Speed_Unit_ADDRESS, RPM_Unit);
        setMotorParameter(0x01, Set_Single_Param, Control_Mode, Speed_Mode);
        setMotorParameter(0x02, Set_Single_Param, Control_Mode, Speed_Mode);        
    }

    void setMotorSpeed(int16_t left_speed, int16_t right_speed) {
        setMotorParameter(0x01, Set_Two_Param, Motion_Control, left_speed);
        setMotorParameter(0x02, Set_Two_Param, Motion_Control, right_speed);
    }

    void getMotorSpeed() {
        getMotorParameter(Motor_Frequency, 0x01);
        getMotorParameter(Motor_Frequency, 0x02);
    }

    void requestMotorStatus() {
        can_msgs::Frame status_request;
        status_request.id = 0x601;
        CANInterfaceManager::sendFrame(status_request);
        status_request.id = 0x602;
        CANInterfaceManager::sendFrame(status_request);
    }

    void getMotorParameter(uint16_t Index, uint8_t canID) {
        can_msgs::Frame wheel_frame;
        wheel_frame.id = Send_Func_CAN_ID|canID;

        wheel_frame.data[0] = Get_Single_Param;
        wheel_frame.data[1] = Index & 0xFF;
        wheel_frame.data[2] = (Index >> 8) & 0xFF;
        wheel_frame.data[3] = 0x00;
        wheel_frame.data[4] = 0x00;
        wheel_frame.data[5] = 0x00;
        wheel_frame.data[6] = 0x00;
        wheel_frame.data[7] = 0x00;

        for (int i = 0; i < 4; i++) {
            std::cout << wheel_frame.data[i] << " ";
        }
        std::cout << std::endl;

        CANInterfaceManager::sendFrame(wheel_frame);
    }

    void setDataInFrame(can_msgs::Frame* frame, const uint8_t* bytes) {
        std::memcpy(&frame->data[4], bytes, 4);
    }

    template<typename T>
    void to_bytes(T value, uint8_t* destination) {
        // 遍历类型 T 的每个字节
        for (size_t i = 0; i < sizeof(T); ++i) {
            destination[i] = (value >> (i * 8)) & 0xFF;
        }
    }

    template<typename T>
    void setMotorParameter(uint8_t canID, uint8_t Cmd, uint16_t Index, T Value) {
        can_msgs::Frame wheel_frame;
        wheel_frame.id = Send_Func_CAN_ID | canID;

        wheel_frame.data[0] = Cmd;
        wheel_frame.data[1] = Index & 0xFF;
        wheel_frame.data[2] = (Index >> 8) & 0xFF;

        // 使用模板函数来处理所有支持的类型
        static_assert(std::is_same<T, float>::value || std::is_same<T, int16_t>::value || std::is_same<T, int>::value ||
                    std::is_same<T, uint8_t>::value || std::is_same<T, uint16_t>::value || std::is_same<T, uint32_t>::value,
                    "Unsupported type for CAN parameter value");

        to_bytes(Value, wheel_frame.data.data() + 4);  // 直接调用泛型转换字节函数

        CANInterfaceManager::sendFrame(wheel_frame);
    }
}