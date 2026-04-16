// swiftpro_kinematics.cpp
// Copyright 2025 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Kinematics node for the UArm Swift Pro.
// Pure computation — does NOT command the arm to move.
//
// Publishes:
//   /swiftpro/end_effector_position  geometry_msgs/PointStamped
//     Live FK result from current joint states. Firmware-frame XYZ (mm).
//
// Services:
//   /swiftpro/compute_fk  swiftpro_resources/srv/ComputeFK
//     Joint angles (firmware degrees) → Cartesian XYZ (mm).
//
//   /swiftpro/compute_ik  swiftpro_resources/srv/ComputeIK
//     Cartesian XYZ (mm) → joint angles (firmware degrees).
//
// ─────────────────────────────────────────────────────────────────────────────
// KINEMATIC MODEL — see kinematics.hpp for full documentation
// ─────────────────────────────────────────────────────────────────────────────
//
// The parallel linkage decouples the forearm world angle from J2.
// Forearm world angle depends ONLY on J3.
// Parameters calibrated from 16-point commanded sweep, RMS = 0.27mm.
//
// Firmware angle conventions (degrees, zero offsets):
//   J1: 90° = arm pointing along +X (forward). azimuth = J1 - 90°.
//   J2: upper arm angle above horizontal.
//   J3: forearm downward angle from horizontal. 0°=horizontal, +°=down.

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "swiftpro_kinematics/kinematics.hpp"
#include "swiftpro_resources/srv/compute_fk.hpp"
#include "swiftpro_resources/srv/compute_ik.hpp"

namespace kin = swiftpro::kinematics;

// ─────────────────────────────────────────────────────────────────────────────
// ROS2 node
// ─────────────────────────────────────────────────────────────────────────────

class SwiftProKinematics : public rclcpp::Node
{
public:
    SwiftProKinematics()
    : Node("swiftpro_kinematics")
    {
        // ── Subscriber: joint states from hardware node ───────────────────────
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10,
            std::bind(&SwiftProKinematics::on_joint_states, this,
                      std::placeholders::_1));

        // ── Publisher: live end effector position ────────────────────────────
        ee_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
            "swiftpro/end_effector_position", 10);

        // ── Service: compute FK ──────────────────────────────────────────────
        fk_srv_ = create_service<swiftpro_resources::srv::ComputeFK>(
            "swiftpro/compute_fk",
            std::bind(&SwiftProKinematics::handle_compute_fk, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ── Service: compute IK ──────────────────────────────────────────────
        ik_srv_ = create_service<swiftpro_resources::srv::ComputeIK>(
            "swiftpro/compute_ik",
            std::bind(&SwiftProKinematics::handle_compute_ik, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
                    "swiftpro_kinematics ready — built %s %s",
                    __DATE__, __TIME__);
        RCLCPP_INFO(get_logger(),
                    "FK params: L1=%.2f L2=%.2f SH_Z=%.2f DX=%.2f DZ=%.2f",
                    kin::L1, kin::L2, kin::SH_Z, kin::DX, kin::DZ);
        RCLCPP_INFO(get_logger(),
                    "IK limits: Z_MIN=%.1f Z_MAX=%.1f J3_MAX=%.1f J2_MIN=%.1f",
                    kin::Z_MIN, kin::Z_MAX, kin::J3_MAX, kin::J2_MIN);
    }

private:
    // ── Joint state callback — publish live FK ────────────────────────────────
    void on_joint_states(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 3) { return; }

        // Hardware node publishes radians — convert to firmware degrees
        const double j1 = msg->position[0] * (180.0 / M_PI);
        const double j2 = msg->position[1] * (180.0 / M_PI);
        const double j3 = msg->position[2] * (180.0 / M_PI);

        double x, y, z;
        kin::fk(j1, j2, j3, x, y, z);

        geometry_msgs::msg::PointStamped pt;
        pt.header.stamp    = msg->header.stamp;
        pt.header.frame_id = "base_link";
        pt.point.x         = x;
        pt.point.y         = y;
        pt.point.z         = z;
        ee_pub_->publish(pt);
    }

    // ── Service: compute FK ───────────────────────────────────────────────────
    void handle_compute_fk(
        const std::shared_ptr<swiftpro_resources::srv::ComputeFK::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::ComputeFK::Response> res)
    {
        kin::fk(req->j1, req->j2, req->j3, res->x, res->y, res->z);
        res->success = true;
        res->message = "OK";
        RCLCPP_DEBUG(get_logger(),
                     "compute_fk: J(%.1f,%.1f,%.1f) → (%.1f,%.1f,%.1f)",
                     req->j1, req->j2, req->j3, res->x, res->y, res->z);
    }

    // ── Service: compute IK ───────────────────────────────────────────────────
    void handle_compute_ik(
        const std::shared_ptr<swiftpro_resources::srv::ComputeIK::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::ComputeIK::Response> res)
    {
        std::string msg;
        res->success = kin::ik(req->x, req->y, req->z,
                               res->j1, res->j2, res->j3, msg);
        res->message = msg;

        if (res->success) {
            RCLCPP_DEBUG(get_logger(),
                         "compute_ik: (%.1f,%.1f,%.1f) → J(%.1f,%.1f,%.1f)",
                         req->x, req->y, req->z, res->j1, res->j2, res->j3);
        } else {
            RCLCPP_WARN(get_logger(),
                        "compute_ik: (%.1f,%.1f,%.1f) failed: %s",
                        req->x, req->y, req->z, msg.c_str());
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr ee_pub_;
    rclcpp::Service<swiftpro_resources::srv::ComputeFK>::SharedPtr fk_srv_;
    rclcpp::Service<swiftpro_resources::srv::ComputeIK>::SharedPtr ik_srv_;
};

// ─────────────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SwiftProKinematics>());
    rclcpp::shutdown();
    return 0;
}
