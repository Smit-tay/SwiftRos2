// swiftpro_kinematics.cpp
// Copyright 2025 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Forward kinematics node for the UArm Swift Pro.
//
// This node subscribes to /joint_states (Joint1, Joint2, Joint3 from the
// hardware node) and computes the Cartesian position of the end effector
// analytically using the Denavit-Hartenberg parameters derived from the
// URDF link geometry.
//
// It publishes the result on /swiftpro/end_effector_position so that any
// consumer (TopicFS, web dashboard, SteampunkClock) can read the current
// tip position without needing to query TF.
//
// It does NOT broadcast TF transforms — that is robot_state_publisher's job.
// These two nodes are fully compatible and complementary:
//   robot_state_publisher  → visualization (RViz, TF tree)
//   swiftpro_kinematics    → analytical FK → Cartesian position topic
//
// Learning note:
//   Forward kinematics answers: "given these joint angles, where is the
//   end effector in Cartesian space?"
//   The approach used here is geometric FK — building up the position
//   by projecting each link length through the accumulated joint angles.
//   This is equivalent to the Denavit-Hartenberg method for this arm
//   topology but expressed more intuitively.

#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// -----------------------------------------------------------------------------
// Link geometry — all values taken directly from swiftpro.xacro joint origins.
// Units: metres, radians.
// -----------------------------------------------------------------------------

// Height of Joint1 (base rotation) above the ground plane
static constexpr double BASE_HEIGHT    = 0.0723;

// Joint1 → Joint2 offset (Joint2 is forward and up from Joint1)
static constexpr double J1_TO_J2_X    = 0.0132;
static constexpr double J1_TO_J2_Z    = 0.0333;

// Upper arm length: Joint2 → Joint3
static constexpr double UPPER_ARM_LEN = 0.14207;

// Forearm length: Joint3 → Joint8 (end effector mount)
static constexpr double FOREARM_LEN   = 0.15852;

// -----------------------------------------------------------------------------

class SwiftProKinematics : public rclcpp::Node
{
public:
  SwiftProKinematics() : Node("swiftpro_kinematics")
  {
    end_effector_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      "swiftpro/end_effector_position", 10);

    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&SwiftProKinematics::on_joint_state, this,
                std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "SwiftPro kinematics node started — publishing FK on "
                "/swiftpro/end_effector_position");
  }

private:
  // ---------------------------------------------------------------------------
  // Retrieve a named joint position from a JointState message.
  // Returns 0.0 and warns (once) if the joint is not present.
  // ---------------------------------------------------------------------------
  double get_joint(
    const sensor_msgs::msg::JointState::SharedPtr& msg,
    const std::string& name) const
  {
    auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end())
    {
      RCLCPP_WARN_ONCE(get_logger(),
                       "Joint '%s' not found in /joint_states", name.c_str());
      return 0.0;
    }
    size_t idx = static_cast<size_t>(std::distance(msg->name.begin(), it));
    if (idx >= msg->position.size())
    {
      RCLCPP_WARN_ONCE(get_logger(),
                       "Joint '%s' has no position value", name.c_str());
      return 0.0;
    }
    return msg->position[idx];
  }

  // ---------------------------------------------------------------------------
  // Forward kinematics callback
  //
  // The UArm Swift Pro is a 3-DOF arm. The base rotates around Z (Joint1).
  // The shoulder (Joint2) and elbow (Joint3) both rotate around Y in the
  // vertical plane of the arm.
  //
  // Step 1: Work in the arm's own vertical plane (ignoring base rotation).
  //         Project upper arm and forearm lengths through the joint angles
  //         to find the end effector position in the arm plane.
  //
  // Step 2: Rotate the arm-plane position around Z by the base angle (Joint1)
  //         to get the final world-frame Cartesian position.
  //
  // The parallel linkage keeps the end effector level — it does not change
  // the tip position, only the orientation. FK position is therefore computed
  // from Joint1/2/3 alone, which is correct.
  // ---------------------------------------------------------------------------
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const double theta1 = get_joint(msg, "Joint1");  // base rotation
    const double theta2 = get_joint(msg, "Joint2");  // shoulder
    const double theta3 = get_joint(msg, "Joint3");  // elbow

    // ── Step 1: position in the arm's vertical plane ─────────────────────
    //
    // Joint2 and Joint3 both rotate around Y.
    // The combined angle at the forearm tip is (theta2 + theta3).
    //
    // Horizontal reach from Joint2 pivot:
    //   x_arm = L_upper * cos(theta2) + L_forearm * cos(theta2 + theta3)
    //
    // Vertical rise from Joint2 pivot:
    //   z_arm = L_upper * sin(theta2) + L_forearm * sin(theta2 + theta3)
    //
    // Note: positive theta2 raises the arm (shoulder elevation),
    //       positive theta3 raises the forearm relative to the upper arm.

    const double cos2  = std::cos(theta2);
    const double sin2  = std::sin(theta2);
    const double cos23 = std::cos(theta2 + theta3);
    const double sin23 = std::sin(theta2 + theta3);

    // Reach and height measured from the Joint2 pivot
    const double reach_from_j2 = UPPER_ARM_LEN * cos2 + FOREARM_LEN * cos23;
    const double rise_from_j2  = UPPER_ARM_LEN * sin2 + FOREARM_LEN * sin23;

    // Total horizontal reach from the arm centre axis (add the J1→J2 offset)
    const double total_reach = J1_TO_J2_X + reach_from_j2;

    // Total height above ground
    const double z = BASE_HEIGHT + J1_TO_J2_Z + rise_from_j2;

    // ── Step 2: rotate around Z by theta1 ────────────────────────────────
    //
    // The arm plane is aligned with the X axis at theta1 = 0.
    // Rotating by theta1 gives us world-frame X and Y.

    const double x = total_reach * std::cos(theta1);
    const double y = total_reach * std::sin(theta1);

    // ── Publish ───────────────────────────────────────────────────────────
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp    = msg->header.stamp;
    pt.header.frame_id = "Base";
    pt.point.x = x;
    pt.point.y = y;
    pt.point.z = z;

    end_effector_pub_->publish(pt);

    RCLCPP_DEBUG(get_logger(),
                 "FK: θ1=%.3f θ2=%.3f θ3=%.3f → x=%.4f y=%.4f z=%.4f",
                 theta1, theta2, theta3, x, y, z);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr end_effector_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwiftProKinematics>());
  rclcpp::shutdown();
  return 0;
}
