#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <vector>

class SwiftProKinematics : public rclcpp::Node {
public:
    SwiftProKinematics() : Node("swiftpro_kinematics") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&SwiftProKinematics::jointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "SwiftPro Kinematics Node started in namespace: %s", this->get_namespace());
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Get joint positions
        double joint1_pos = getJointPosition(msg, "Joint1");
        double joint2_pos = getJointPosition(msg, "Joint2");
        double joint3_pos = getJointPosition(msg, "Joint3");
        double joint4_pos = getJointPosition(msg, "Joint4");
        double joint5_pos = getJointPosition(msg, "Joint5");

        // Parallelogram: Link4 stays level, Joint5 mirrors Joint4 inversely
        double expected_joint5_pos = -joint4_pos;  // Adjust based on SwiftPro
        if (std::abs(joint5_pos - expected_joint5_pos) > 0.01) {
            joint5_pos = expected_joint5_pos;
            RCLCPP_DEBUG(this->get_logger(), "Adjusted Joint5 to %f to match Joint4", joint5_pos);
        }

        // Base transforms
        publishTransform("Base", "Link1", 0.0, 0.0, 0.0723, 0.0, 0.0, joint1_pos);  // Joint1 (z-axis)

        // Parallelogram transforms
        publishTransform("Link1", "Link2", 0.0132, 0.0, 0.0333, 0.0, joint2_pos, 0.0);  // Joint2
        publishTransform("Link2", "Link4", 0.0, 0.0, 0.14207, 0.0, joint4_pos, 0.0);  // Joint4 (Link4-A)
        publishTransform("Link1", "Link5", -0.0215, 0.0, 0.05001, 0.0, joint5_pos, 0.0);  // Joint5

        // Link4 (A) to Link4 (B) - triangle’s upper rear, approximate length
        double link4_a_to_b_x = 0.0375;  // Tune: distance from A to B
        double link4_a_to_b_z = 0.0275;  // Tune: height difference
        publishTransform("Link4", "Link4_B", link4_a_to_b_x, 0.0, link4_a_to_b_z, 0.0, 0.0, 0.0);

        // Link4 (B) to Link5 - closes the parallelogram
        double link4_b_to_link5_x = -0.03;  // Tune: back from B to Link5’s end
        publishTransform("Link4_B", "Link5", link4_b_to_link5_x, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Link4 (A) to Link4 (C) - triangle’s upper forward
        double link4_a_to_c_x = 0.0375;  // Tune: distance from A to C
        double link4_a_to_c_z = 0.0275;  // Tune: height difference
        publishTransform("Link4", "Link4_C", link4_a_to_c_x, 0.0, link4_a_to_c_z, 0.0, 0.0, 0.0);

        // End effector: Link4 (C) -> Link9 -> Link8 (fixing Xacro’s Link3 -> Link8)
        publishTransform("Link4_C", "Link9", 0.0, 0.0, 0.0, 0.0, joint3_pos, 0.0);  // Joint3 moves Link9
        publishTransform("Link9", "Link8", 0.02741, 0.0, 0.02703, 0.0, getJointPosition(msg, "Joint9"), 0.0);  // Joint9 moves Link8

        // Wrist (Link6, Link7)
        publishTransform("Link1", "Link6", 0.0132, 0.0, 0.0333, 0.0, getJointPosition(msg, "Joint6"), 0.0);  // Joint6
        publishTransform("Link6", "Link7", -0.0455, 0.0, -0.00301, 0.0, getJointPosition(msg, "Joint7"), 0.0);  // Joint7
    }

    double getJointPosition(const sensor_msgs::msg::JointState::SharedPtr msg, const std::string& joint_name) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
        if (it == msg->name.end()) {
            RCLCPP_WARN_ONCE(this->get_logger(), "%s not found in joint states.", joint_name.c_str());
            return 0.0;
        }
        return msg->position[std::distance(msg->name.begin(), it)];
    }

    void publishTransform(const std::string& parent, const std::string& child, 
                          double x, double y, double z, double roll, double pitch, double yaw) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = parent;
        t.child_frame_id = child;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwiftProKinematics>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}