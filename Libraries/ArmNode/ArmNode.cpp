#include "ArmNode.h"

#include "../Inverse_Kinematics/Inverse_Kinematics.h"
#include "../Utilities/Utilities.h"

#include <cmath>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <SDL2/SDL.h>

namespace {
constexpr float PI = 3.14159265358979323846f;

bool probe_i2c_address(const std::string& device, std::uint8_t address) {
    const int fd = ::open(device.c_str(), O_RDWR);
    if (fd < 0) {
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        ::close(fd);
        return false;
    }

    std::uint8_t reg = 0x00;
    std::uint8_t value = 0;
    const bool ok = (::write(fd, &reg, 1) == 1) && (::read(fd, &value, 1) == 1);
    ::close(fd);
    return ok;
}

void scan_i2c_bus(const std::string& device) {
    const int fd = ::open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open " << device << " for scanning" << std::endl;
        return;
    }

    std::cout << "Scanning " << device << "..." << std::endl;
    bool found_any = false;
    for (std::uint8_t address = 0x03; address <= 0x77; ++address) {
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            continue;
        }

        std::uint8_t reg = 0x00;
        std::uint8_t value = 0;
        if ((::write(fd, &reg, 1) == 1) && (::read(fd, &value, 1) == 1)) {
            std::cout << "Found device at 0x" << std::hex
                      << static_cast<int>(address) << std::dec << std::endl;
            found_any = true;
        }
    }

    if (!found_any) {
        std::cout << "No I2C devices found" << std::endl;
    }
    ::close(fd);
}
}  // namespace

ArmNode::ArmNode()
    : Node("arm_node") {
    i2c_device_ =
        declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
    const int i2c_address = declare_parameter<int>("i2c_address", i2c_address_);
    if (i2c_address < 0x03 || i2c_address > 0x77) {
        throw std::invalid_argument("i2c_address must be between 0x03 and 0x77");
    }

    i2c_address_ = static_cast<std::uint8_t>(i2c_address);
    pwm_ = std::make_unique<PCA9685>(i2c_address_, i2c_device_);
    initialize_hardware();

    joint_targets_publisher_ =
        create_publisher<sensor_msgs::msg::JointState>("arm/joint_targets", 10);
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        [this]() { control_tick(); });

    std::cout << "ArmNode started. PID: " << getpid() << std::endl;
}

ArmNode::~ArmNode() {
    running_ = false;
    control_timer_.reset();
    stop_actuators();
}

void ArmNode::initialize_hardware() {
    if (!probe_i2c_address(i2c_device_, i2c_address_)) {
        std::cerr << "I2C address 0x" << std::hex
                  << static_cast<int>(i2c_address_) << std::dec
                  << " not visible on " << i2c_device_ << std::endl;
        scan_i2c_bus(i2c_device_);
        throw std::runtime_error("PCA9685 was not found on the I2C bus");
    }

    if (!pwm_->open()) {
        throw std::runtime_error("Failed to open PCA9685 on " + i2c_device_);
    }
    hardware_open_ = true;
    std::cout << "PCA9685 initialized at 50Hz." << std::endl;

    if (!controller_.checkController()) {
        throw std::runtime_error("No game controller was found");
    }
    sleep(1);
}

void ArmNode::control_tick() {
    if (!running_) {
        return;
    }

    if (!controller_.getProgramState()) {
        stop();
        return;
    }

    controller_.updateAxes();

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_JOYBUTTONDOWN ||
            event.type == SDL_CONTROLLERBUTTONDOWN) {
            controller_.handleJoyButtons(event);
        }
        if (event.type == SDL_QUIT || event.type == SDL_JOYDEVICEREMOVED) {
            stop();
            return;
        }
    }

    const std::int16_t lsx = controller_.getLSX();
    const std::int16_t lsy = controller_.getLSY();
    if (std::abs(lsx) > DEADZONE || std::abs(lsy) > DEADZONE) {
        const float vector_ls = controller_.getLSVector() / VECTOR_MAX;
        const float angle_ls = controller_.getLSAngle();

        const float x_delta = (std::cos(angle_ls) * vector_ls) / 750.0f;
        const float y_delta = (std::sin(angle_ls) * vector_ls) / 750.0f;
        state_.x = constrain(state_.x + x_delta, -0.3, 0.3);
        state_.y = constrain(state_.y + y_delta, -0.3, 0.3);
    }
    const float z_delta =
        -(controller_.getLTCurve() / 1000.0f) +
        (controller_.getRTCurve() / 1000.0f);
    state_.z = constrain(state_.z + z_delta, -0.3, 0.3);

    const std::int16_t rsx = controller_.getRSX();
    const std::int16_t rsy = controller_.getRSY();
    if (std::abs(rsx) > DEADZONE || std::abs(rsy) > DEADZONE) {
        const float vector_rs = controller_.getRSVector() / VECTOR_MAX;
        const float angle_rs = controller_.getRSAngle();
        const float pitch_delta = (std::cos(angle_rs) * vector_rs) / 15.0f;
        const float yaw_delta = (std::sin(angle_rs) * vector_rs) / 15.0f;

        state_.pitch = constrain(state_.pitch + pitch_delta, 0, 2 * PI);
        state_.yaw = constrain(state_.yaw + yaw_delta, 0, 2 * PI);
    }

    state_.roll = controller_.getBMPValue();

    const std::vector<double> solutions = IK_solver(
        state_.x,
        state_.y,
        state_.z,
        state_.roll,
        state_.pitch,
        state_.yaw);

    bool solution_found = solutions.size() == 5;
    for (double solution : solutions) {
        if (solution < 0.0) {
            solution_found = false;
        }
    }

    if (solution_found) {
        constexpr double smoothness = 0.3;
        const std::uint16_t angles[] = {
            static_cast<std::uint16_t>(solutions[0] + 135),
            static_cast<std::uint16_t>(solutions[1] + 45),
            static_cast<std::uint16_t>(solutions[2] + 90),
            static_cast<std::uint16_t>(solutions[3] + 90),
            static_cast<std::uint16_t>(solutions[4] + 90),
        };
        const std::uint8_t channels[] = {
            BASE, SHOULDER, UPPER_ARM, FOREARM, WRIST,
        };
        const std::uint8_t servo_types[] = {
            MS62_SERVO, MS62_SERVO_A, DM996_SERVO, DM996_SERVO, DM996_SERVO,
        };
        bool servo_command_ok = true;
        for (std::size_t i = 0; i < 5; ++i) {
            if (!pwm_->setSmoothServoAngle(
                    channels[i], servo_types[i], angles[i], smoothness)) {
                servo_command_ok = false;
            }
            usleep(20);
        }
        if (!servo_command_ok) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Failed to write one or more servo targets");
        }

        sensor_msgs::msg::JointState targets;
        targets.header.stamp = now();
        targets.name = {
            "base", "shoulder", "upper_arm", "forearm", "wrist"};
        targets.position.reserve(solutions.size());
        for (double solution : solutions) {
            targets.position.push_back(solution * PI / 180.0);
        }
        joint_targets_publisher_->publish(targets);
    }

}

void ArmNode::stop() {
    running_ = false;
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void ArmNode::stop_actuators() {
    if (!hardware_open_ || !pwm_) {
        return;
    }

    for (int i = 0; i < 50; ++i) {
        pwm_->setSmoothServoAngle(BASE, MS62_SERVO, 135, 2);
        usleep(20);
        pwm_->setSmoothServoAngle(SHOULDER, MS62_SERVO_A, 141, 2);
        usleep(20);
        pwm_->setSmoothServoAngle(UPPER_ARM, DM996_SERVO, 60, 2);
        usleep(20);
        pwm_->setSmoothServoAngle(FOREARM, DM996_SERVO, 90, 2);
        usleep(20);
        pwm_->setSmoothServoAngle(WRIST, DM996_SERVO, 90, 2);
        usleep(20);
        pwm_->setSmoothServoAngle(FINGER, DM996_SERVO, 90, 2);
        usleep(100000);
    }

    for (int channel = 0; channel < 16; ++channel) {
        if (!pwm_->setPWM(channel, 0, 4096)) {
            std::cerr << "Failed to turn off PCA9685 channel " << channel
                      << std::endl;
        }
    }
    if (!pwm_->sleep()) {
        std::cerr << "Failed to put PCA9685 to sleep" << std::endl;
    }
    hardware_open_ = false;
    std::cout << "Goodbye." << std::endl;
}
