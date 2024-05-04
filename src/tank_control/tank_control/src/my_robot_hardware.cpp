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
    void valueToData(uint8_t* data, T value) {
        static_assert(std::is_standard_layout<T>::value, "Type T must be standard layout");

        T tmp = value;  // Create a temporary variable to ensure value is not modified
        uint8_t* bytes = reinterpret_cast<uint8_t*>(&tmp);
        for (size_t i = 0; i < sizeof(T); i++) {
            data[i] = bytes[i];  // Copy byte by byte, lower byte first
        }
    }

    // Log the data for debugging
    void logData(const uint8_t* data, size_t size) {
        std::cout << "Data to be sent: ";
        for (size_t i = 0; i < size; ++i) {
            std::cout << std::hex << static_cast<int>(data[i]) << " ";
        }
        std::cout << std::dec << std::endl; // Switch back to decimal for normal output
    }

    template<typename T>
    void setMotorParameter(uint8_t canID, uint8_t Cmd, uint16_t Index, T Value) {
        can_msgs::Frame wheel_frame;
        wheel_frame.id = Send_Func_CAN_ID | canID;
        wheel_frame.dlc = 8;

        wheel_frame.data[0] = Cmd;
        wheel_frame.data[1] = Index & 0xFF;
        wheel_frame.data[2] = (Index >> 8) & 0xFF;

        // Convert Value to bytes and store in data[4] to data[7]
        valueToData<T>(wheel_frame.data.data() + 4, Value);
        // Log the data before sending
        // logData(wheel_frame.data.data(), wheel_frame.data.size());  // Use .data() and .size() for boost::array

        // Send the CAN frame
        CANInterfaceManager::sendFrame(wheel_frame);
    }
}