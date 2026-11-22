#include "Libraries/ArmNode/ArmNode.h"

#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int exit_code = 0;

    try {
        auto node = std::make_shared<ArmNode>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        std::cerr << "ArmNode failed: " << error.what() << std::endl;
        exit_code = 1;
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    return exit_code;
}
