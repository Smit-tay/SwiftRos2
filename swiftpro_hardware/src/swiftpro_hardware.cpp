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
//     /swiftpro/polar            geometry_msgs/Point     (stretch, rotation, height)
//     /swiftpro/pump_status      std_msgs/Int8           (0=stop, 1=working, 2=caught)
//     /swiftpro/gripper_status   std_msgs/Int8           (0=stop, 1=working, 2=caught)
//     /swiftpro/connected        std_msgs/Bool
//     /swiftpro/is_moving        std_msgs/Bool
//     /swiftpro/limit_switch     std_msgs/Bool
//     /swiftpro/mode             std_msgs/Int8           (0=general,1=laser,2=3Dprint,3=pen/gripper)
//
//   Actions:
//     /swiftpro/move_arm         swiftpro_resources/action/MoveArm
//       Goal:     x, y, z (mm), speed (mm/min)
//       Feedback: current_x, current_y, current_z (mm) at 10 Hz while moving
//       Result:   success, error_code, message — position verified within move_tolerance_mm
//       Cancel:   flush pending commands, command current position (best-effort stop)
//
//   Services:
//     /swiftpro/set_pump         swiftpro_resources/srv/SetPump
//     /swiftpro/set_gripper      swiftpro_resources/srv/SetGripper
//     /swiftpro/reset            swiftpro_resources/srv/Reset
//     /swiftpro/set_wrist        swiftpro_resources/srv/SetWrist
//     /swiftpro/set_servo_attach swiftpro_resources/srv/SetServoAttach
//     /swiftpro/set_mode         swiftpro_resources/srv/SetMode
//     /swiftpro/set_polar        swiftpro_resources/srv/SetPolar
//     /swiftpro/set_servo_angle  swiftpro_resources/srv/SetServoAngle
//     /swiftpro/set_buzzer       swiftpro_resources/srv/SetBuzzer
//     /swiftpro/set_acceleration swiftpro_resources/srv/SetAcceleration
//     /swiftpro/set_digital_output     swiftpro_resources/srv/SetDigitalOutput
//     /swiftpro/set_digital_direction  swiftpro_resources/srv/SetDigitalDirection
//     /swiftpro/get_digital      swiftpro_resources/srv/GetDigital
//     /swiftpro/get_analog       swiftpro_resources/srv/GetAnalog
//     /swiftpro/get_servo_attach swiftpro_resources/srv/GetServoAttach
//     /swiftpro/smooth_move      swiftpro_resources/srv/SmoothMove
//
// Joint names Joint1/Joint2/Joint3 match the URDF joint names in swiftpro.xacro exactly.
// robot_state_publisher computes Joint4-Joint8 via <mimic> tags.
// The swiftpro_kinematics node computes end effector FK from these joints.

// Standard library
#include <atomic>
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
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int8.hpp>

// uarm
#include "uarm/uarm.h"

// Project messages
#include "swiftpro_resources/action/move_arm.hpp"
#include "swiftpro_resources/srv/get_analog.hpp"
#include "swiftpro_resources/srv/get_digital.hpp"
#include "swiftpro_resources/srv/get_servo_attach.hpp"
#include "swiftpro_resources/srv/reset.hpp"
#include "swiftpro_resources/srv/set_acceleration.hpp"
#include "swiftpro_resources/srv/set_buzzer.hpp"
#include "swiftpro_resources/srv/set_digital_direction.hpp"
#include "swiftpro_resources/srv/set_digital_output.hpp"
#include "swiftpro_resources/srv/set_gripper.hpp"
#include "swiftpro_resources/srv/set_mode.hpp"
#include "swiftpro_resources/srv/set_polar.hpp"
#include "swiftpro_resources/srv/set_pump.hpp"
#include "swiftpro_resources/srv/set_servo_angle.hpp"
#include "swiftpro_resources/srv/set_servo_angles.hpp"
#include "swiftpro_resources/srv/set_servo_attach.hpp"
#include "swiftpro_resources/srv/set_wrist.hpp"
#include "swiftpro_resources/srv/smooth_move.hpp"

// Kinematics — FK/IK shared with swiftpro_kinematics node
#include "swiftpro_kinematics/kinematics.hpp"


// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

static const std::vector<std::string> DRIVEN_JOINT_NAMES = {
  "Joint1",
  "Joint2",
  "Joint3"
};

static constexpr double DEG_TO_RAD  = M_PI / 180.0;
// default_speed = 10000 defined in uarm/uarm.h

// -----------------------------------------------------------------------------

class SwiftProHardware : public rclcpp::Node
{
public:
  using MoveArm           = swiftpro_resources::action::MoveArm;
  using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

  SwiftProHardware()
  : Node("swiftpro_hardware")
  , swift_(nullptr)
  {
    instance_ = this;

    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<double>("position_report_interval", 0.1);
    declare_parameter<double>("joint_state_publish_rate", 10.0);
    declare_parameter<double>("move_tolerance_mm", 15.0);

    port_                     = get_parameter("port").as_string();
    baudrate_                 = get_parameter("baudrate").as_int();
    position_report_interval_ = get_parameter("position_report_interval").as_double();
    const double publish_rate = get_parameter("joint_state_publish_rate").as_double();
    move_tolerance_mm_        = get_parameter("move_tolerance_mm").as_double();

    // ── Publishers ──────────────────────────────────────────────────────────
    joint_state_pub_    = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    position_pub_       = create_publisher<geometry_msgs::msg::Point>("swiftpro/position", 10);
    polar_pub_          = create_publisher<geometry_msgs::msg::Point>("swiftpro/polar", 10);
    pump_status_pub_    = create_publisher<std_msgs::msg::Int8>("swiftpro/pump_status", 10);
    gripper_status_pub_ = create_publisher<std_msgs::msg::Int8>("swiftpro/gripper_status", 10);
    connected_pub_      = create_publisher<std_msgs::msg::Bool>("swiftpro/connected", 10);
    is_moving_pub_      = create_publisher<std_msgs::msg::Bool>("swiftpro/is_moving", 10);
    limit_switch_pub_   = create_publisher<std_msgs::msg::Bool>("swiftpro/limit_switch", 10);
    mode_pub_           = create_publisher<std_msgs::msg::Int8>("swiftpro/mode", 10);

    // ── Action server ────────────────────────────────────────────────────────
    move_arm_server_ = rclcpp_action::create_server<MoveArm>(
      this,
      "swiftpro/move_arm",
      std::bind(&SwiftProHardware::handle_goal,     this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&SwiftProHardware::handle_cancel,   this,
                std::placeholders::_1),
      std::bind(&SwiftProHardware::handle_accepted, this,
                std::placeholders::_1));

    // ── Services ────────────────────────────────────────────────────────────
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

    set_wrist_srv_ = create_service<swiftpro_resources::srv::SetWrist>(
      "swiftpro/set_wrist",
      std::bind(&SwiftProHardware::handle_set_wrist, this,
                std::placeholders::_1, std::placeholders::_2));

    set_servo_attach_srv_ = create_service<swiftpro_resources::srv::SetServoAttach>(
      "swiftpro/set_servo_attach",
      std::bind(&SwiftProHardware::handle_set_servo_attach, this,
                std::placeholders::_1, std::placeholders::_2));

    set_mode_srv_ = create_service<swiftpro_resources::srv::SetMode>(
      "swiftpro/set_mode",
      std::bind(&SwiftProHardware::handle_set_mode, this,
                std::placeholders::_1, std::placeholders::_2));

    set_polar_srv_ = create_service<swiftpro_resources::srv::SetPolar>(
      "swiftpro/set_polar",
      std::bind(&SwiftProHardware::handle_set_polar, this,
                std::placeholders::_1, std::placeholders::_2));

    set_servo_angle_srv_ = create_service<swiftpro_resources::srv::SetServoAngle>(
      "swiftpro/set_servo_angle",
      std::bind(&SwiftProHardware::handle_set_servo_angle, this,
                std::placeholders::_1, std::placeholders::_2));

    set_servo_angles_srv_ = create_service<swiftpro_resources::srv::SetServoAngles>(
      "swiftpro/set_servo_angles",
      std::bind(&SwiftProHardware::handle_set_servo_angles, this,
                std::placeholders::_1, std::placeholders::_2));

    set_buzzer_srv_ = create_service<swiftpro_resources::srv::SetBuzzer>(
      "swiftpro/set_buzzer",
      std::bind(&SwiftProHardware::handle_set_buzzer, this,
                std::placeholders::_1, std::placeholders::_2));

    set_acceleration_srv_ = create_service<swiftpro_resources::srv::SetAcceleration>(
      "swiftpro/set_acceleration",
      std::bind(&SwiftProHardware::handle_set_acceleration, this,
                std::placeholders::_1, std::placeholders::_2));

    set_digital_output_srv_ = create_service<swiftpro_resources::srv::SetDigitalOutput>(
      "swiftpro/set_digital_output",
      std::bind(&SwiftProHardware::handle_set_digital_output, this,
                std::placeholders::_1, std::placeholders::_2));

    set_digital_direction_srv_ = create_service<swiftpro_resources::srv::SetDigitalDirection>(
      "swiftpro/set_digital_direction",
      std::bind(&SwiftProHardware::handle_set_digital_direction, this,
                std::placeholders::_1, std::placeholders::_2));

    get_digital_srv_ = create_service<swiftpro_resources::srv::GetDigital>(
      "swiftpro/get_digital",
      std::bind(&SwiftProHardware::handle_get_digital, this,
                std::placeholders::_1, std::placeholders::_2));

    get_analog_srv_ = create_service<swiftpro_resources::srv::GetAnalog>(
      "swiftpro/get_analog",
      std::bind(&SwiftProHardware::handle_get_analog, this,
                std::placeholders::_1, std::placeholders::_2));

    get_servo_attach_srv_ = create_service<swiftpro_resources::srv::GetServoAttach>(
      "swiftpro/get_servo_attach",
      std::bind(&SwiftProHardware::handle_get_servo_attach, this,
                std::placeholders::_1, std::placeholders::_2));

    smooth_move_srv_ = create_service<swiftpro_resources::srv::SmoothMove>(
      "swiftpro/smooth_move",
      std::bind(&SwiftProHardware::handle_smooth_move, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
                "SwiftPro hardware node —   Jack Sidman Smith - built %s %s",
                __DATE__, __TIME__);

    // ── Connect to arm ───────────────────────────────────────────────────────
    connect();

    // ── Timers ───────────────────────────────────────────────────────────────
    const auto publish_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / publish_rate));

    publish_timer_ = create_wall_timer(
      publish_period,
      std::bind(&SwiftProHardware::publish_state, this));

    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&SwiftProHardware::check_connection, this));

    RCLCPP_INFO(get_logger(),
                "SwiftPro hardware node ready — port=%s rate=%.1fHz tolerance=%.1fmm",
                port_.c_str(), publish_rate, move_tolerance_mm_);
  }

  ~SwiftProHardware()
  {
    if (swift_ && swift_->connected) { swift_->disconnect(); }
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

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      swift_->register_report_position_callback(
        SwiftProHardware::position_callback);
      swift_->register_limit_switch_callback(
        SwiftProHardware::limit_switch_callback);

      const int ret = swift_->set_report_position(
        static_cast<float>(position_report_interval_), true, 5.0f);
      RCLCPP_DEBUG(get_logger(),
                   "connect: set_report_position returned %d", ret);

      std::string* info = swift_->get_device_info();
      if (info)
      {
        RCLCPP_INFO(get_logger(), "Device type     : %s", info[0].c_str());
        RCLCPP_INFO(get_logger(), "Hardware version: %s", info[1].c_str());
        RCLCPP_INFO(get_logger(), "Firmware version: %s", info[2].c_str());
        RCLCPP_INFO(get_logger(), "API version     : %s", info[3].c_str());
        RCLCPP_INFO(get_logger(), "Device unique   : %s", info[4].c_str());
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
  // Callback trampolines
  // ---------------------------------------------------------------------------

  static void position_callback(std::vector<float> position)
  {
    if (instance_) { instance_->on_position_report(position); }
  }

  static void limit_switch_callback(bool state)
  {
    if (instance_) { instance_->on_limit_switch(state); }
  }

  void on_position_report(const std::vector<float>& position)
  {
    if (position.size() < 3) { return; }
    std::lock_guard<std::mutex> lock(position_mutex_);
    current_position_ = position;
  }

  void on_limit_switch(bool state)
  {
    std_msgs::msg::Bool msg;
    msg.data = state;
    limit_switch_pub_->publish(msg);
  }

  // ---------------------------------------------------------------------------
  // State publishing
  // ---------------------------------------------------------------------------

  void publish_state()
  {
    if (!swift_ || !swift_->connected) { return; }

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

    const auto polar = swift_->get_polar();
    if (polar.size() >= 3)
    {
      geometry_msgs::msg::Point pt;
      pt.x = static_cast<double>(polar[0]);
      pt.y = static_cast<double>(polar[1]);
      pt.z = static_cast<double>(polar[2]);
      polar_pub_->publish(pt);
    }

    const int pump = swift_->get_pump_status();
    if (pump >= 0)
    {
      std_msgs::msg::Int8 msg;
      msg.data = static_cast<int8_t>(pump);
      pump_status_pub_->publish(msg);
    }

    const int gripper = swift_->get_gripper_catch();
    if (gripper >= 0)
    {
      std_msgs::msg::Int8 msg;
      msg.data = static_cast<int8_t>(gripper);
      gripper_status_pub_->publish(msg);
    }

    const int moving = swift_->get_is_moving();
    if (moving >= 0)
    {
      std_msgs::msg::Bool msg;
      msg.data = (moving == 1);
      is_moving_pub_->publish(msg);
    }

    const int mode = swift_->get_mode();
    if (mode >= 0)
    {
      std_msgs::msg::Int8 msg;
      msg.data = static_cast<int8_t>(mode);
      mode_pub_->publish(msg);
    }
  }

  // ---------------------------------------------------------------------------
  // Action server — MoveArm
  // ---------------------------------------------------------------------------

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const MoveArm::Goal> goal)
  {
    if (!swift_ || !swift_->connected)
    {
      RCLCPP_WARN(get_logger(), "move_arm: rejected — arm not connected");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal_active_.exchange(true))
    {
      RCLCPP_WARN(get_logger(),
                  "move_arm: rejected — previous goal still executing");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_logger(),
                "move_arm: goal received x=%.1f y=%.1f z=%.1f speed=%.0f",
                goal->x, goal->y, goal->z, goal->speed);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveArm> /*goal_handle*/)
  {
    RCLCPP_INFO(get_logger(), "move_arm: cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    std::thread(
      std::bind(&SwiftProHardware::execute_move_arm, this, goal_handle)
    ).detach();
  }

  void execute_move_arm(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    const auto goal     = goal_handle->get_goal();
    auto       feedback = std::make_shared<MoveArm::Feedback>();
    auto       result   = std::make_shared<MoveArm::Result>();

    swift_->set_position(
      goal->x, goal->y, goal->z,
      static_cast<long>(goal->speed),
      false, false);

    constexpr auto poll_interval = std::chrono::milliseconds(100);
    constexpr int  max_polls     = 300;

    for (int i = 0; i < max_polls; ++i)
    {
      std::this_thread::sleep_for(poll_interval);

      if (goal_handle->is_canceling())
      {
        swift_->flush_cmd();
        {
          std::lock_guard<std::mutex> lock(position_mutex_);
          if (current_position_.size() >= 3)
          {
            swift_->set_position(
              current_position_[0], current_position_[1], current_position_[2],
              static_cast<long>(goal->speed), false, false);
          }
        }
        result->success    = false;
        result->error_code = -3;
        result->message    = "Cancelled";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "move_arm: cancelled");
        goal_active_.store(false);
        return;
      }

      {
        std::lock_guard<std::mutex> lock(position_mutex_);
        if (current_position_.size() >= 3)
        {
          feedback->current_x = current_position_[0];
          feedback->current_y = current_position_[1];
          feedback->current_z = current_position_[2];
          goal_handle->publish_feedback(feedback);
        }
      }

      if (swift_->get_is_moving() == 0) { break; }
    }

    float actual_x{0.0f}, actual_y{0.0f}, actual_z{0.0f};
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      if (current_position_.size() >= 3)
      {
        actual_x = current_position_[0];
        actual_y = current_position_[1];
        actual_z = current_position_[2];
      }
    }

    const float dx   = std::abs(actual_x - goal->x);
    const float dy   = std::abs(actual_y - goal->y);
    const float dz   = std::abs(actual_z - goal->z);
    const float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    const bool  ok   = (dist <= static_cast<float>(move_tolerance_mm_));

    if (goal_handle->is_active())
    {
      result->success    = ok;
      result->error_code = ok ? 0 : -2;
      if (ok) {
        result->message = "OK";
      } else {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
          "Position tolerance exceeded: dist=%.1fmm (tol=%.1fmm) "
          "actual=(%.1f,%.1f,%.1f)",
          dist, move_tolerance_mm_, actual_x, actual_y, actual_z);
        result->message = buf;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(),
                  "move_arm: %s dist=%.1fmm actual=(%.1f,%.1f,%.1f)",
                  ok ? "succeeded" : "failed",
                  dist, actual_x, actual_y, actual_z);
    }
    goal_active_.store(false);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — motion
  // ---------------------------------------------------------------------------

  void handle_set_polar(
    const std::shared_ptr<swiftpro_resources::srv::SetPolar::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetPolar::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_polar(
      req->stretch, req->rotation, req->height,
      static_cast<long>(req->speed), req->relative, req->wait);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(),
                "set_polar: stretch=%.1f rotation=%.1f height=%.1f result=%d",
                req->stretch, req->rotation, req->height, r);
  }

  void handle_set_wrist(
    const std::shared_ptr<swiftpro_resources::srv::SetWrist::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetWrist::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_wrist(req->angle, static_cast<long>(req->speed));
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(), "set_wrist: angle=%.1f result=%d", req->angle, r);
  }

  void handle_set_servo_angle(
    const std::shared_ptr<swiftpro_resources::srv::SetServoAngle::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetServoAngle::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_servo_angle(
      req->servo_id, req->angle, static_cast<long>(req->speed), req->wait);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(),
                "set_servo_angle: servo=%d angle=%.1f result=%d",
                req->servo_id, req->angle, r);
  }

  void handle_set_servo_angles(
    const std::shared_ptr<swiftpro_resources::srv::SetServoAngles::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetServoAngles::Response> res)
  {
    RCLCPP_INFO(get_logger(),
                "set_servo_angles: j1=%.1f j2=%.1f j3=%.1f speed=%d wait=%d",
                req->j1, req->j2, req->j3, req->speed, req->wait);

    if (!swift_ || !swift_->connected) {
      RCLCPP_WARN(get_logger(), "set_servo_angles: arm not connected");
      res->success = false; return;
    }

    const long spd = (req->speed > 0) ? static_cast<long>(req->speed) : default_speed;
    bool ok = true;
    int  r;

    if (req->j1 >= 0.0f) {
      r = swift_->set_servo_angle(0, req->j1, spd, false);
      RCLCPP_INFO(get_logger(),
                  "set_servo_angles: j1=%.1f speed=%ld result=%d", req->j1, spd, r);
    }
    if (req->j2 >= 0.0f) {
      r = swift_->set_servo_angle(1, req->j2, spd, false);
      RCLCPP_INFO(get_logger(),
                  "set_servo_angles: j2=%.1f speed=%ld result=%d", req->j2, spd, r);
    }
    if (req->j3 >= 0.0f) {
      r = swift_->set_servo_angle(2, req->j3, spd, false);
      RCLCPP_INFO(get_logger(),
                  "set_servo_angles: j3=%.1f speed=%ld result=%d", req->j3, spd, r);
    }

    if (req->wait && ok)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      constexpr auto timeout   = std::chrono::seconds(15);
      constexpr auto poll_step = std::chrono::milliseconds(100);
      const auto     start     = std::chrono::steady_clock::now();
      while (true) {
        if (swift_->get_is_moving() == 0) { break; }
        if (std::chrono::steady_clock::now() - start > timeout) {
          RCLCPP_WARN(get_logger(), "set_servo_angles: wait timeout");
          ok = false; break;
        }
        std::this_thread::sleep_for(poll_step);
      }
    }

    res->success = ok;
    RCLCPP_INFO(get_logger(), "set_servo_angles: complete success=%s",
                ok ? "true" : "false");
  }

  // ---------------------------------------------------------------------------
  // Service handler — smooth_move
  //
  // Moves the arm from current position to target via an automatic arc:
  //
  //   current → via → target
  //
  // The via point is the XY midpoint of current and target, lifted in Z:
  //   via_x = (cur_x + tgt_x) / 2
  //   via_y = (cur_y + tgt_y) / 2
  //   via_z = max(cur_z, tgt_z) + lift   (default lift = 50mm)
  //
  // Each leg (current→via, via→target) is executed as a direct interpolated
  // move using sine ease-in/ease-out in joint space. Step count scales with
  // leg distance so short legs are fast and long legs are smooth.
  //
  // This produces a natural pick-and-place arc without the noise of the
  // firmware's continuous servo correction trajectory planner.
  // ---------------------------------------------------------------------------

  // Execute one interpolated segment: current angles → target angles.
  // Steps scale with tip distance for consistent apparent speed.
  void execute_segment(double s_j1, double s_j2, double s_j3,
                       double t_j1, double t_j2, double t_j3,
                       double speed_mms)
  {
    namespace kin = swiftpro::kinematics;

    double sx, sy, sz, tx, ty, tz;
    kin::fk(s_j1, s_j2, s_j3, sx, sy, sz);
    kin::fk(t_j1, t_j2, t_j3, tx, ty, tz);

    const double dist  = std::sqrt((tx-sx)*(tx-sx) +
                                   (ty-sy)*(ty-sy) +
                                   (tz-sz)*(tz-sz));
    const double dur   = std::max(dist / speed_mms, 0.2);
    const int    steps = static_cast<int>(
                           std::clamp(dist / 25.0, 4.0, 12.0));
    const int    dwell = static_cast<int>((dur / steps) * 1e6);

    // Joint speed: scale with largest joint delta so servos track the steps
    const double max_j_delta = std::max({std::abs(t_j1-s_j1),
                                         std::abs(t_j2-s_j2),
                                         std::abs(t_j3-s_j3)});
    const long SPD = static_cast<long>(std::clamp(max_j_delta * 80.0, 200.0, 2000.0));

    for (int i = 1; i <= steps; ++i)
    {
      const double u = static_cast<double>(i) / steps;
      const double t = 0.5 * (1.0 - std::cos(M_PI * u));

      const double cmd_j1 = s_j1 + t*(t_j1-s_j1);
      const double cmd_j2 = s_j2 + t*(t_j2-s_j2);
      const double raw_j3 = s_j3 + t*(t_j3-s_j3);
      // Safety clamp — enforce linkage constraints at every step
      const double cmd_j3 = std::max(raw_j3,
                              swiftpro::kinematics::j3_safe_min(cmd_j2));
      if (cmd_j3 != raw_j3) {
        RCLCPP_WARN_ONCE(get_logger(),
          "execute_segment: J3 clamped %.1f→%.1f at J2=%.1f",
          raw_j3, cmd_j3, cmd_j2);
      }

      swift_->set_servo_angle(0, static_cast<float>(cmd_j1), SPD, false);
      swift_->set_servo_angle(1, static_cast<float>(cmd_j2), SPD, false);
      swift_->set_servo_angle(2, static_cast<float>(cmd_j3), SPD, false);

      std::this_thread::sleep_for(std::chrono::microseconds(dwell));
    }
  }

  // ---------------------------------------------------------------------------
  // Service handler — smooth_move
  //
  // Moves the arm from current position to target via an automatic arc:
  //
  //   current → via → target
  //
  // The via point is the XY midpoint of current and target, lifted in Z:
  //   via_x = (cur_x + tgt_x) / 2
  //   via_y = (cur_y + tgt_y) / 2
  //   via_z = max(cur_z, tgt_z) + lift   (default lift = 50mm)
  //
  // Each leg (current→via, via→target) is executed as a direct interpolated
  // move using sine ease-in/ease-out in joint space. Step count scales with
  // leg distance so short legs are fast and long legs are smooth.
  //
  // This produces a natural pick-and-place arc without the noise of the
  // firmware's continuous servo correction trajectory planner.
  // ---------------------------------------------------------------------------
  void handle_smooth_move(
    const std::shared_ptr<swiftpro_resources::srv::SmoothMove::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SmoothMove::Response> res)
  {
    namespace kin = swiftpro::kinematics;

    if (!swift_ || !swift_->connected) {
      res->success = false;
      res->message = "Arm not connected";
      return;
    }

    const double speed = (req->speed > 0.0) ? req->speed : 150.0;

    // ── Current joint angles ─────────────────────────────────────────────────
    const auto angles = swift_->get_servo_angle();
    if (angles.size() < 3) {
      res->success = false;
      res->message = "Failed to read current joint angles";
      return;
    }
    const double s_j1 = static_cast<double>(angles[0]);
    const double s_j2 = static_cast<double>(angles[1]);
    const double s_j3 = static_cast<double>(angles[2]);

    // ── IK for target ────────────────────────────────────────────────────────
    double t_j1, t_j2, t_j3;
    std::string msg;
    if (!kin::ik(req->x, req->y, req->z, t_j1, t_j2, t_j3, msg)) {
      res->success = false;
      res->message = "Target IK failed: " + msg;
      RCLCPP_WARN(get_logger(), "smooth_move target IK: %s", msg.c_str());
      return;
    }

    // ── Safe-corridor waypoint planner ───────────────────────────────────────
    //
    // Walks J2 from start to target in PLAN_STEPS equal increments.
    // At each step, J3 is the linear interpolation of start→target J3,
    // clamped up to j3_safe_min(j2) if needed.
    // J1 is linearly interpolated throughout.
    //
    // This mirrors the manual stepped approach that was physically verified:
    //   reset(J2=90,J3=0) → (60,10) → (40,20) → (30,26) → (21,32) ✓
    //
    // Waypoints where the clamped J3 equals the linear J3 are collapsed —
    // only waypoints where the clamp actually changes the path are kept,
    // plus the final target.

    constexpr int PLAN_STEPS = 20;

    struct Waypoint { double j1, j2, j3; };
    std::vector<Waypoint> waypoints;

    double prev_j1 = s_j1, prev_j2 = s_j2, prev_j3 = s_j3;

    for (int i = 1; i <= PLAN_STEPS; ++i) {
      const double t   = static_cast<double>(i) / PLAN_STEPS;
      const double j1  = s_j1 + t * (t_j1 - s_j1);
      const double j2  = s_j2 + t * (t_j2 - s_j2);
      const double j3l = s_j3 + t * (t_j3 - s_j3);          // linear J3
      const double j3  = std::max(j3l, kin::j3_safe_min(j2)); // clamped J3

      // Only emit waypoint if position changed meaningfully
      const double dj = std::abs(j2 - prev_j2) + std::abs(j3 - prev_j3);
      if (dj > 0.5 || i == PLAN_STEPS) {
        waypoints.push_back({j1, j2, j3});
        prev_j1 = j1; prev_j2 = j2; prev_j3 = j3;
      }
    }

    RCLCPP_INFO(get_logger(),
                "smooth_move: (%.1f,%.1f,%.1f)→(%.1f,%.1f,%.1f) via %zu waypoints",
                s_j1, s_j2, s_j3, t_j1, t_j2, t_j3, waypoints.size());

    // ── Execute waypoint sequence ────────────────────────────────────────────
    double cur_j1 = s_j1, cur_j2 = s_j2, cur_j3 = s_j3;
    for (const auto& wp : waypoints) {
      execute_segment(cur_j1, cur_j2, cur_j3, wp.j1, wp.j2, wp.j3, speed);
      cur_j1 = wp.j1; cur_j2 = wp.j2; cur_j3 = wp.j3;
    }

    // ── Optional wait for settle ─────────────────────────────────────────────
    if (req->wait) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      constexpr auto timeout   = std::chrono::seconds(10);
      constexpr auto poll_step = std::chrono::milliseconds(50);
      const auto     start     = std::chrono::steady_clock::now();
      while (swift_->get_is_moving() == 1) {
        if (std::chrono::steady_clock::now() - start > timeout) {
          RCLCPP_WARN(get_logger(), "smooth_move: wait timeout");
          break;
        }
        std::this_thread::sleep_for(poll_step);
      }
    }

    res->success = true;
    res->message = "OK";
  }

  void handle_reset(
    const std::shared_ptr<swiftpro_resources::srv::Reset::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::Reset::Response> res)
  {
    if (!swift_ || !swift_->connected) { res->success = false; return; }
    const float x = (req->x > 0.0f)  ? req->x : 200.0f;
    const float y = (req->y != 0.0f) ? req->y :   0.0f;
    const float z = (req->z > 0.0f)  ? req->z : 150.0f;
    swift_->reset(static_cast<long>(req->speed), 10.0f, x, y, z);
    res->success = true;
    RCLCPP_INFO(get_logger(),
                "reset: complete — x=%.1f y=%.1f z=%.1f", x, y, z);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — end effectors
  // ---------------------------------------------------------------------------

  void handle_set_pump(
    const std::shared_ptr<swiftpro_resources::srv::SetPump::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetPump::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_pump(req->on);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(), "set_pump: %s result=%d",
                req->on ? "ON" : "OFF", r);
  }

  void handle_set_gripper(
    const std::shared_ptr<swiftpro_resources::srv::SetGripper::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetGripper::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_gripper(req->catch_object);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(), "set_gripper: %s result=%d",
                req->catch_object ? "CATCH" : "RELEASE", r);
  }

  void handle_set_buzzer(
    const std::shared_ptr<swiftpro_resources::srv::SetBuzzer::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetBuzzer::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_buzzer(req->frequency, req->duration);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(), "set_buzzer: freq=%d dur=%.1f result=%d",
                req->frequency, req->duration, r);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — servo management
  // ---------------------------------------------------------------------------

  void handle_set_servo_attach(
    const std::shared_ptr<swiftpro_resources::srv::SetServoAttach::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetServoAttach::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = req->attach
      ? swift_->set_servo_attach(req->servo_id)
      : swift_->set_servo_detach(req->servo_id);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(), "set_servo_attach: servo=%d attach=%s result=%d",
                req->servo_id, req->attach ? "true" : "false", r);
  }

  void handle_get_servo_attach(
    const std::shared_ptr<swiftpro_resources::srv::GetServoAttach::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::GetServoAttach::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->attached = false; res->success = false; return;
    }
    const int r   = swift_->get_servo_attach(req->servo_id);
    res->success  = (r >= 0);
    res->attached = (r == 0);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — configuration
  // ---------------------------------------------------------------------------

  void handle_set_mode(
    const std::shared_ptr<swiftpro_resources::srv::SetMode::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetMode::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_mode(req->mode);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(), "set_mode: mode=%d result=%d", req->mode, r);
  }

  void handle_set_acceleration(
    const std::shared_ptr<swiftpro_resources::srv::SetAcceleration::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetAcceleration::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_acceleration(req->acc);
    res->success = (r == 0); res->error_code = r;
    RCLCPP_INFO(get_logger(), "set_acceleration: acc=%.2f result=%d", req->acc, r);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — GPIO
  // ---------------------------------------------------------------------------

  void handle_set_digital_output(
    const std::shared_ptr<swiftpro_resources::srv::SetDigitalOutput::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetDigitalOutput::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_digital_output(req->pin, req->value);
    res->success = (r == 0); res->error_code = r;
  }

  void handle_set_digital_direction(
    const std::shared_ptr<swiftpro_resources::srv::SetDigitalDirection::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::SetDigitalDirection::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->success = false; res->error_code = -1; return;
    }
    const int r = swift_->set_digital_direction(req->pin, req->value);
    res->success = (r == 0); res->error_code = r;
  }

  void handle_get_digital(
    const std::shared_ptr<swiftpro_resources::srv::GetDigital::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::GetDigital::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->value = -1; res->success = false; return;
    }
    const int r = swift_->get_digital(req->pin);
    res->value   = r;
    res->success = (r >= 0);
  }

  void handle_get_analog(
    const std::shared_ptr<swiftpro_resources::srv::GetAnalog::Request>  req,
          std::shared_ptr<swiftpro_resources::srv::GetAnalog::Response> res)
  {
    if (!swift_ || !swift_->connected) {
      res->value = -1; res->success = false; return;
    }
    const int r = swift_->get_analog(req->pin);
    res->value   = r;
    res->success = (r >= 0);
  }

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  std::unique_ptr<uarm::Swift> swift_;
  std::string                  port_;
  int                          baudrate_;
  double                       position_report_interval_;
  double                       move_tolerance_mm_;

  std::mutex         position_mutex_;
  std::vector<float> current_position_;

  static SwiftProHardware* instance_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr     position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr     polar_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           pump_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           gripper_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           is_moving_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           limit_switch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           mode_pub_;

  rclcpp_action::Server<MoveArm>::SharedPtr move_arm_server_;
  std::atomic<bool>                         goal_active_{false};
  std::thread                               execute_thread_;

  rclcpp::Service<swiftpro_resources::srv::SetPump>::SharedPtr             set_pump_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetGripper>::SharedPtr          set_gripper_srv_;
  rclcpp::Service<swiftpro_resources::srv::Reset>::SharedPtr               reset_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetWrist>::SharedPtr            set_wrist_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetServoAttach>::SharedPtr      set_servo_attach_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetMode>::SharedPtr             set_mode_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetPolar>::SharedPtr            set_polar_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetServoAngle>::SharedPtr       set_servo_angle_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetServoAngles>::SharedPtr      set_servo_angles_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetBuzzer>::SharedPtr           set_buzzer_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetAcceleration>::SharedPtr     set_acceleration_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetDigitalOutput>::SharedPtr    set_digital_output_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetDigitalDirection>::SharedPtr set_digital_direction_srv_;
  rclcpp::Service<swiftpro_resources::srv::GetDigital>::SharedPtr          get_digital_srv_;
  rclcpp::Service<swiftpro_resources::srv::GetAnalog>::SharedPtr           get_analog_srv_;
  rclcpp::Service<swiftpro_resources::srv::GetServoAttach>::SharedPtr      get_servo_attach_srv_;
  rclcpp::Service<swiftpro_resources::srv::SmoothMove>::SharedPtr          smooth_move_srv_;

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
