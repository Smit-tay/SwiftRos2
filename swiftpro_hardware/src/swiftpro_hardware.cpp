// swiftpro_hardware.cpp
// Copyright 2025 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Hardware interface node for the UArm Swift Pro robotic arm.
//
// Connects to the arm via USB serial and provides:
//   Publishers:
//     /joint_states              sensor_msgs/JointState  (Joint1, Joint2, Joint3)
//     /swiftpro/position         geometry_msgs/Point     (raw XYZ from arm firmware)
//     /swiftpro/pump_status      std_msgs/Bool
//     /swiftpro/connected        std_msgs/Bool
//
//   Services:
//     /swiftpro/move_to          swiftpro_resources/srv/MoveTo
//     /swiftpro/set_pump         swiftpro_resources/srv/SetPump
//     /swiftpro/set_gripper      swiftpro_resources/srv/SetGripper
//     /swiftpro/reset            swiftpro_resources/srv/Reset
//
// Joint names Joint1/Joint2/Joint3 match the URDF exactly.
// robot_state_publisher computes Joint4-Joint8 via <mimic> tags.
// The swiftpro_kinematics node computes end effector FK from these joints.

// Standard library
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ROS2
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

// uarm
#include "uarm/uarm.h"

// Project messages
#include "swiftpro_resources/srv/move_to.hpp"
#include "swiftpro_resources/srv/reset.hpp"
#include "swiftpro_resources/srv/set_gripper.hpp"
#include "swiftpro_resources/srv/set_pump.hpp"

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// Joint names must match the URDF joint names in swiftpro.xacro exactly.
// robot_state_publisher uses these names to look up mimic relationships.
static const std::vector<std::string> DRIVEN_JOINT_NAMES = {
  "Joint1",   // base rotation   — servo 0
  "Joint2",   // shoulder        — servo 1
  "Joint3"    // elbow/forearm   — servo 2
};

static constexpr double DEG_TO_RAD = M_PI / 180.0;

// -----------------------------------------------------------------------------

class SwiftProHardware : public rclcpp::Node
{
public:
  SwiftProHardware()
  : Node("swiftpro_hardware")
  , swift_(nullptr)
  {
    // Must be set before connect() — position_callback() uses it
    instance_ = this;

    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<double>("position_report_interval", 0.1);
    declare_parameter<double>("joint_state_publish_rate", 10.0);

    port_                      = get_parameter("port").as_string();
    baudrate_                  = get_parameter("baudrate").as_int();
    position_report_interval_  = get_parameter("position_report_interval").as_double();
    const double publish_rate  = get_parameter("joint_state_publish_rate").as_double();

    // ── Publishers ──────────────────────────────────────────────────────────
    joint_state_pub_  = create_publisher<sensor_msgs::msg::JointState>(
                          "joint_states", 10);
    position_pub_     = create_publisher<geometry_msgs::msg::Point>(
                          "swiftpro/position", 10);
    pump_status_pub_  = create_publisher<std_msgs::msg::Bool>(
                          "swiftpro/pump_status", 10);
    connected_pub_    = create_publisher<std_msgs::msg::Bool>(
                          "swiftpro/connected", 10);

    // ── Services ────────────────────────────────────────────────────────────
    move_to_srv_ = create_service<swiftpro_resources::srv::MoveTo>(
      "swiftpro/move_to",
      std::bind(&SwiftProHardware::handle_move_to, this,
                std::placeholders::_1, std::placeholders::_2));

    set_pump_srv_ = create_service<swiftpro_resources::srv::SetPump>(
      "swiftpro/set_pump",
      std::bind(&SwiftProHardware::handle_set_pump, this,
                std::placeholders::_1, std::placeholders::_2));

    set_gripper_srv_ = create_service<swiftpro_resources::srv::SetGripper>(
      "swiftpro/set_gripper",
      std::bind(&SwiftProHardware::handle_set_gripper, this,
                std::placeholders::_1, std::placeholders::_2));

    reset_srv_ = create_service<swiftpro_resources::srv::Reset>(
      "swiftpro/reset",
      std::bind(&SwiftProHardware::handle_reset, this,
                std::placeholders::_1, std::placeholders::_2));

    // ── Connect to arm ───────────────────────────────────────────────────────
    connect();

    // ── Timers ───────────────────────────────────────────────────────────────
    const auto publish_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate));

    publish_timer_ = create_wall_timer(
      publish_period,
      std::bind(&SwiftProHardware::publish_state, this));

    // Connection watchdog — reconnects automatically if arm is unplugged
    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&SwiftProHardware::check_connection, this));

    RCLCPP_INFO(get_logger(),
                "SwiftPro hardware node ready — port=%s rate=%.1fHz",
                port_.c_str(), publish_rate);
  }

  ~SwiftProHardware()
  {
    if (swift_ && swift_->connected)
    {
      swift_->disconnect();
    }
    instance_ = nullptr;
  }

private:
  // ---------------------------------------------------------------------------
  // Connection management
  // ---------------------------------------------------------------------------

  void connect()
  {
    try
    {
      swift_ = std::make_unique<uarm::Swift>(
        port_, static_cast<uint32_t>(baudrate_));

      if (!swift_->connected)
      {
        RCLCPP_ERROR(get_logger(),
                     "connect: failed to connect on %s", port_.c_str());
        swift_.reset();
        return;
      }

      // Allow arm firmware to stabilise before enabling position reporting
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // Register position report handler.
      // The uarm API requires a raw C function pointer — we use a static
      // trampoline that forwards to the instance method.
      swift_->register_report_position_callback(
        SwiftProHardware::position_callback);

      const int ret = swift_->set_report_position(
        static_cast<float>(position_report_interval_), true, 5.0f);
      RCLCPP_DEBUG(get_logger(),
                   "connect: set_report_position returned %d", ret);

      // Log device info
      std::string* info = swift_->get_device_info();
      if (info)
      {
        RCLCPP_INFO(get_logger(), "Device type     : %s", info[0].c_str());
        RCLCPP_INFO(get_logger(), "Hardware version: %s", info[1].c_str());
        RCLCPP_INFO(get_logger(), "Firmware version: %s", info[2].c_str());
      }

      RCLCPP_INFO(get_logger(),
                  "connect: connected to UArm Swift Pro on %s", port_.c_str());
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "connect: exception: %s", e.what());
      swift_.reset();
    }
  }

  void check_connection()
  {
    const bool connected = swift_ && swift_->connected;
    if (!connected)
    {
      RCLCPP_WARN(get_logger(),
                  "check_connection: arm disconnected — attempting reconnect");
      connect();
    }

    std_msgs::msg::Bool msg;
    msg.data = connected;
    connected_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // Position callback trampoline
  //
  // The uarm library calls a raw C function pointer from its own thread.
  // This static function forwards the call to the instance method.
  // instance_ is set before connect() is called and cleared in the destructor.
  // ---------------------------------------------------------------------------

  static void position_callback(std::vector<float> position)
  {
    if (instance_)
    {
      instance_->on_position_report(position);
    }
  }

  void on_position_report(const std::vector<float>& position)
  {
    if (position.size() < 3) { return; }
    std::lock_guard<std::mutex> lock(position_mutex_);
    current_position_ = position;
  }

  // ---------------------------------------------------------------------------
  // State publishing
  //
  // Called by publish_timer_ at joint_state_publish_rate Hz.
  // Publishes:
  //   /joint_states  — Joint1/2/3 servo angles converted to radians
  //   /swiftpro/position — raw XYZ from firmware position report callback
  //   /swiftpro/pump_status — pump on/off
  // ---------------------------------------------------------------------------

  void publish_state()
  {
    if (!swift_ || !swift_->connected) { return; }

    // ── JointState — the three driven joints only ──────────────────────────
    // Joint names must match swiftpro.xacro exactly so that
    // robot_state_publisher can apply the <mimic> constraints for Joint4-8.
    const auto angles = swift_->get_servo_angle();
    if (angles.size() >= 3)
    {
      sensor_msgs::msg::JointState js;
      js.header.stamp = now();
      js.name         = DRIVEN_JOINT_NAMES;
      js.position     = {
        angles[0] * DEG_TO_RAD,
        angles[1] * DEG_TO_RAD,
        angles[2] * DEG_TO_RAD
      };
      joint_state_pub_->publish(js);
    }

    // ── XYZ position from firmware ─────────────────────────────────────────
    // This is the arm's own FK estimate — useful for cross-checking against
    // the swiftpro_kinematics node's analytical result.
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      if (current_position_.size() >= 3)
      {
        geometry_msgs::msg::Point pt;
        pt.x = static_cast<double>(current_position_[0]);
        pt.y = static_cast<double>(current_position_[1]);
        pt.z = static_cast<double>(current_position_[2]);
        position_pub_->publish(pt);
      }
    }

    // ── Pump status ────────────────────────────────────────────────────────
    const int pump = swift_->get_pump_status();
    if (pump >= 0)
    {
      std_msgs::msg::Bool msg;
      msg.data = (pump > 0);
      pump_status_pub_->publish(msg);
    }
  }

  // ---------------------------------------------------------------------------
  // Service handlers
  // ---------------------------------------------------------------------------

  void handle_move_to(
    const std::shared_ptr<swiftpro_resources::srv::MoveTo::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::MoveTo::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      response->message    = "Arm not connected";
      return;
    }

    RCLCPP_INFO(get_logger(),
                "move_to: x=%.1f y=%.1f z=%.1f speed=%.0f wait=%s",
                request->x, request->y, request->z, request->speed,
                request->wait ? "true" : "false");

    const int result = swift_->set_position(
      request->x, request->y, request->z,
      static_cast<long>(request->speed),
      false,
      request->wait);

    response->success    = (result == 0);
    response->error_code = result;
    response->message    = (result == 0) ? "OK" : "Movement failed";
  }

  void handle_set_pump(
    const std::shared_ptr<swiftpro_resources::srv::SetPump::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetPump::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result     = swift_->set_pump(request->on);
    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "set_pump: %s result=%d",
                request->on ? "ON" : "OFF", result);
  }

  void handle_set_gripper(
    const std::shared_ptr<swiftpro_resources::srv::SetGripper::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetGripper::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result     = swift_->set_gripper(request->catch_object);
    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "set_gripper: %s result=%d",
                request->catch_object ? "CATCH" : "RELEASE", result);
  }

  void handle_reset(
    const std::shared_ptr<swiftpro_resources::srv::Reset::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::Reset::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success = false;
      return;
    }

    // Use requested position if provided, otherwise fall back to safe defaults
    const float x = (request->x > 0.0f)    ? request->x : 200.0f;
    const float y = (request->y != 0.0f)   ? request->y : 0.0f;
    const float z = (request->z > 0.0f)    ? request->z : 150.0f;

    swift_->reset(static_cast<long>(request->speed), 10.0f, x, y, z);

    response->success = true;
    RCLCPP_INFO(get_logger(),
                "reset: complete — x=%.1f y=%.1f z=%.1f", x, y, z);
  }

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  // uarm driver
  std::unique_ptr<uarm::Swift> swift_;
  std::string                  port_;
  int                          baudrate_;
  double                       position_report_interval_;

  // Position state — written from uarm callback thread, read from timer thread
  std::mutex             position_mutex_;
  std::vector<float>     current_position_;

  // Static trampoline target — one instance only (uarm API limitation)
  static SwiftProHardware* instance_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr     position_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           pump_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           connected_pub_;

  // Services
  rclcpp::Service<swiftpro_resources::srv::MoveTo>::SharedPtr     move_to_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetPump>::SharedPtr    set_pump_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetGripper>::SharedPtr set_gripper_srv_;
  rclcpp::Service<swiftpro_resources::srv::Reset>::SharedPtr      reset_srv_;

  // Timers
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

SwiftProHardware* SwiftProHardware::instance_ = nullptr;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SwiftProHardware>());
  rclcpp::shutdown();
  return 0;
}
