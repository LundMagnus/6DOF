#include "ArmNode.h"

#include "../Inverse_Kinematics/Inverse_Kinematics.h"
#include "../Utilities/Utilities.h"

#include <cmath>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include <SDL2/SDL.h>
#include <ftxui/component/component.hpp>
#include <ftxui/dom/table.hpp>

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
    : Node("arm_node"),
      screen_(ftxui::ScreenInteractive::TerminalOutput()) {
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

    rclcpp::on_shutdown([this]() {
        running_.store(false);
        if (ui_active_.load()) {
            screen_.ExitLoopClosure()();
        }
    });

    std::cout << "ArmNode started. PID: " << getpid() << std::endl;
}

ArmNode::~ArmNode() {
    running_.store(false);
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
    if (!running_.load()) {
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

    float angle_ls = 0.0f;
    const std::int16_t lsx = controller_.getLSX();
    const std::int16_t lsy = controller_.getLSY();
    if (std::abs(lsx) > DEADZONE || std::abs(lsy) > DEADZONE) {
        const float vector_ls = controller_.getLSVector() / VECTOR_MAX;
        angle_ls = controller_.getLSAngle();

        const float x_delta = (std::cos(angle_ls) * vector_ls) / 750.0f;
        const float y_delta = (std::sin(angle_ls) * vector_ls) / 750.0f;
        state_.x.store(constrain(state_.x.load() + x_delta, -0.3, 0.3));
        state_.y.store(constrain(state_.y.load() + y_delta, -0.3, 0.3));
        state_.x_delta.store(x_delta);
        state_.y_delta.store(y_delta);
    } else {
        state_.x_delta.store(0.0f);
        state_.y_delta.store(0.0f);
    }

    const float z_delta =
        -(controller_.getLTCurve() / 1000.0f) +
        (controller_.getRTCurve() / 1000.0f);
    state_.z.store(constrain(state_.z.load() + z_delta, -0.3, 0.3));
    state_.z_delta.store(z_delta);

    const std::int16_t rsx = controller_.getRSX();
    const std::int16_t rsy = controller_.getRSY();
    if (std::abs(rsx) > DEADZONE || std::abs(rsy) > DEADZONE) {
        const float vector_rs = controller_.getRSVector() / VECTOR_MAX;
        const float angle_rs = controller_.getRSAngle();
        const float pitch_delta = (std::cos(angle_rs) * vector_rs) / 15.0f;
        const float yaw_delta = (std::sin(angle_rs) * vector_rs) / 15.0f;

        state_.pitch.store(
            constrain(state_.pitch.load() + pitch_delta, 0, 2 * PI));
        state_.yaw.store(constrain(state_.yaw.load() + yaw_delta, 0, 2 * PI));
        state_.pitch_delta.store(pitch_delta);
        state_.yaw_delta.store(yaw_delta);
    } else {
        state_.pitch_delta.store(0.0f);
        state_.yaw_delta.store(0.0f);
    }

    state_.roll.store(controller_.getBMPValue());

    const std::vector<double> solutions = IK_solver(
        state_.x.load(),
        state_.y.load(),
        state_.z.load(),
        state_.roll.load(),
        state_.pitch.load(),
        state_.yaw.load());

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
        state_.angle0.store(static_cast<float>(angles[0]));
        state_.angle1.store(static_cast<float>(angles[1]));
        state_.angle2.store(static_cast<float>(angles[2]));
        state_.angle3.store(static_cast<float>(angles[3]));
        state_.angle4.store(static_cast<float>(angles[4]));

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

    screen_.PostEvent(ftxui::Event::Custom);
}

void ArmNode::stop() {
    running_.store(false);
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

void ArmNode::run_ui() {
    using namespace ftxui;
    ui_active_.store(true);
    if (running_.load()) {
        auto renderer = Renderer([this]() { return render_ui(); });
        screen_.Loop(renderer);
    }
    ui_active_.store(false);
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

ftxui::Element ArmNode::render_ui() {
    using namespace ftxui;
    std::vector<std::vector<std::string>> data = {
        {"Name", "Value", "Delta"},
        {"x", std::to_string(state_.x.load()),
         std::to_string(state_.x_delta.load())},
        {"y", std::to_string(state_.y.load()),
         std::to_string(state_.y_delta.load())},
        {"z", std::to_string(state_.z.load()),
         std::to_string(state_.z_delta.load())},
        {"roll", std::to_string(state_.roll.load()),
         std::to_string(state_.roll_delta.load())},
        {"pitch", std::to_string(state_.pitch.load()),
         std::to_string(state_.pitch_delta.load())},
        {"yaw", std::to_string(state_.yaw.load()),
         std::to_string(state_.yaw_delta.load())},
    };

    auto pose_table = Table(data);
    pose_table.SelectRow(0).Decorate(bold | color(Color::Cyan));
    pose_table.SelectColumn(0).Decorate(color(Color::Yellow));
    pose_table.SelectAll().Border(LIGHT);
    pose_table.SelectAll().Separator(LIGHT);

    std::vector<std::vector<std::string>> angles = {
        {"Servo", "Angle"},
        {"MS62_1", std::to_string(state_.angle0.load())},
        {"MS62_2", std::to_string(state_.angle1.load())},
        {"DM996_1", std::to_string(state_.angle2.load())},
        {"DM996_2", std::to_string(state_.angle3.load())},
        {"DM996_3", std::to_string(state_.angle4.load())},
        {"DM996_4", std::to_string(state_.angle5.load())},
    };

    auto servo_table = Table(angles);
    servo_table.SelectRow(0).Decorate(bold | color(Color::Cyan));
    servo_table.SelectColumn(0).Decorate(color(Color::Yellow));
    servo_table.SelectAll().Border(LIGHT);
    servo_table.SelectAll().Separator(LIGHT);

    return vbox({
        hbox({
            pose_table.Render(),
            separator(),
            servo_table.Render(),
        }) | center,
        text("Add IK solution here eventually.") | center,
    });
}
