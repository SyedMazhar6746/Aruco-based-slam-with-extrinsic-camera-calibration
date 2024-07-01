#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <signal.h> // For signal handling

class VelocityCommand : public rclcpp::Node {
public:
    VelocityCommand() : Node("velocity_command") {
        pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/kobuki/commands/wheel_velocities", 10);

        // Set initial parameters
        this->declare_parameter<double>("left_wheel", 4.0);
        this->declare_parameter<double>("right_wheel", 4.0);
        left_wheel = this->get_parameter("left_wheel").as_double();
        right_wheel = this->get_parameter("right_wheel").as_double();

        // Register the signal handler for Ctrl+C
        signal(SIGINT, signalHandler);
    }

    void velMove() {
        rclcpp::Rate rate(100);
        auto move_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        while (rclcpp::ok()) {
            // Get parameters dynamically
            left_wheel = this->get_parameter("left_wheel").as_double();
            right_wheel = this->get_parameter("right_wheel").as_double();

            move_msg->data = {left_wheel, right_wheel};
            pub->publish(move_msg);
            rate.sleep();
        }
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub;
    double left_wheel, right_wheel;

    static void signalHandler(int sig) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ctrl+C pressed. Stopping the loop...");
        rclcpp::shutdown();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VelocityCommand>();
    node->velMove();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
