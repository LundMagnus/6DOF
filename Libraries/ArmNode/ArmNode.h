#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "../Controller/Controller.h"
#include "../PCA9685/PCA9685.h"

class ArmNode final : public rclcpp::Node {
public:
    ArmNode();
    ~ArmNode() override;

private:
    struct State {
        float x{0.0f};
        float y{0.0f};
        float z{0.3f};
        float roll{0.0f};
        float pitch{0.0f};
        float yaw{0.0f};
    };

    void initialize_hardware();
    void control_tick();
    void stop();
    void stop_actuators();

    static constexpr int BASE = 0;
    static constexpr int SHOULDER = 1;
    static constexpr int UPPER_ARM = 2;
    static constexpr int FOREARM = 3;
    static constexpr int WRIST = 4;
    static constexpr int FINGER = 5;

    static constexpr int MS62_SERVO = 0;
    static constexpr int DM996_SERVO = 1;
    static constexpr int MS62_SERVO_A = 2;

    static constexpr int DEADZONE = 5000;
    static constexpr int VECTOR_MAX = 37000;

    std::string i2c_device_;
    std::uint8_t i2c_address_{0x40};
    std::unique_ptr<PCA9685> pwm_;
    Controller controller_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_targets_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    State state_;
    bool running_{true};
    bool hardware_open_{false};
};
