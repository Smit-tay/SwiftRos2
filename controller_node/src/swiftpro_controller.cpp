#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class UArmController : public rclcpp::Node {
 public:
  UArmController() : Node("swiftpro_controller") {
    RCLCPP_INFO(this->get_logger(), "uArm SwiftPro Controller Node Started");

    // Example subscription (e.g., to receive movement commands)
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "uarm/command", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
          RCLCPP_INFO(this->get_logger(), "Received command: '%s'", msg->data.c_str());
          // Process command here
        });

    // Example publisher (e.g., to publish uArm state)
    publisher_ = this->create_publisher<std_msgs::msg::String>("swiftpro/state", 10);

    // Timer to publish state periodically
    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      auto message = std_msgs::msg::String();
      message.data = "uArm SwiftPro is operational";
      publisher_->publish(message);
    });
  }

 private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UArmController>());
  rclcpp::shutdown();
  return 0;
}
