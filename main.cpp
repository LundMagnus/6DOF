#include "Libraries/ArmNode/ArmNode.h"

#include <exception>
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<ArmNode>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        std::thread executor_thread([&executor]() {
            executor.spin();
        });

        node->run_ui();
        executor.cancel();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        executor_thread.join();
    } catch (const std::exception& error) {
        std::cerr << "ArmNode failed: " << error.what() << std::endl;
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
        return 1;
    }

    return 0;
}
