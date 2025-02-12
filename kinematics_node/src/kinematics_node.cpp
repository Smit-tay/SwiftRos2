#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class KinematicsNode : public rclcpp::Node {
 public:
  KinematicsNode() : Node("kinematics_node") {
    // Subscribe to raw joint states
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&KinematicsNode::jointStateCallback, this, std::placeholders::_1));

    // Publish corrected joint states
    joint_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/corrected_joint_states", 10);
  }

 private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    dumpJointState(msg);

    auto corrected_msg = *msg;
    /*
        // Find Joint2 and apply corrections to Joint5 and Joint6
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == "joint2") {
            double joint2_angle = msg->position[i];

            // Apply a simple offset for the parallel linkage
            for (size_t j = 0; j < msg->name.size(); ++j) {
              if (msg->name[j] == "joint5") {
                corrected_msg.position[j] = joint2_angle + 0.01;  // Example 1cm offset
              } else if (msg->name[j] == "joint6") {
                corrected_msg.position[j] = joint2_angle + 0.01;
              }
            }
            break;
          }
        }
        */
    joint_pub_->publish(corrected_msg);
  }

  void dumpJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received JointState message:");

    // Print header (timestamp and frame_id)
    RCLCPP_INFO(this->get_logger(), "  Header:");
    RCLCPP_INFO(this->get_logger(), "    Stamp: %u.%09u", msg->header.stamp.sec,
                msg->header.stamp.nanosec);
    RCLCPP_INFO(this->get_logger(), "    Frame ID: %s", msg->header.frame_id.c_str());

    // Print joint names
    RCLCPP_INFO(this->get_logger(), "  Joint Names:");
    for (size_t i = 0; i < msg->name.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "    [%zu]: %s", i, msg->name[i].c_str());
    }

    // Print positions
    RCLCPP_INFO(this->get_logger(), "  Joint Positions:");
    for (size_t i = 0; i < msg->position.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "    [%zu]: %f", i, msg->position[i]);
    }

    // Print velocities (if available)
    if (!msg->velocity.empty()) {
      RCLCPP_INFO(this->get_logger(), "  Joint Velocities:");
      for (size_t i = 0; i < msg->velocity.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "    [%zu]: %f", i, msg->velocity[i]);
      }
    }

    // Print efforts (if available)
    if (!msg->effort.empty()) {
      RCLCPP_INFO(this->get_logger(), "  Joint Efforts:");
      for (size_t i = 0; i < msg->effort.size(); i++) {
        RCLCPP_INFO(this->get_logger(), "    [%zu]: %f", i, msg->effort[i]);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
