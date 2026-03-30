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
//
// Joint names Joint1/Joint2/Joint3 match the URDF joint names in swiftpro.xacro exactly.
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
#include "swiftpro_resources/srv/set_servo_attach.hpp"
#include "swiftpro_resources/srv/set_wrist.hpp"

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
  using MoveArm        = swiftpro_resources::action::MoveArm;
  using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

  SwiftProHardware()
  : Node("swiftpro_hardware")
  , swift_(nullptr)
  {
    // Must be set before connect() — callbacks use it
    instance_ = this;

    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<double>("position_report_interval", 0.1);
    declare_parameter<double>("joint_state_publish_rate", 10.0);
    declare_parameter<double>("move_tolerance_mm", 15.0);

    port_                      = get_parameter("port").as_string();
    baudrate_                  = get_parameter("baudrate").as_int();
    position_report_interval_  = get_parameter("position_report_interval").as_double();
    const double publish_rate  = get_parameter("joint_state_publish_rate").as_double();
    move_tolerance_mm_         = get_parameter("move_tolerance_mm").as_double();

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
    // Each accepted goal runs execute_move_arm in its own thread so the action
    // server remains responsive to new goals and cancels during execution.
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

    RCLCPP_INFO(get_logger(),
                "SwiftPro hardware node —   Jack Sidman Smith - built %s %s", __DATE__, __TIME__);
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
                "SwiftPro hardware node ready — port=%s rate=%.1fHz tolerance=%.1fmm",
                port_.c_str(), publish_rate, move_tolerance_mm_);
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

      // Register callbacks.
      // The uarm API requires raw C function pointers — static trampolines
      // forward to instance methods. instance_ is set before connect() is called.
      swift_->register_report_position_callback(
        SwiftProHardware::position_callback);
      swift_->register_limit_switch_callback(
        SwiftProHardware::limit_switch_callback);

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
  //
  // The uarm library calls raw C function pointers from its own thread.
  // Static functions forward to the single node instance.
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
  //
  // Called by publish_timer_ at joint_state_publish_rate Hz.
  // ---------------------------------------------------------------------------

  void publish_state()
  {
    if (!swift_ || !swift_->connected) { return; }

    // ── JointState — the three driven joints only ──────────────────────────
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

    // ── Polar position ─────────────────────────────────────────────────────
    const auto polar = swift_->get_polar();
    if (polar.size() >= 3)
    {
      geometry_msgs::msg::Point pt;
      pt.x = static_cast<double>(polar[0]);   // stretch
      pt.y = static_cast<double>(polar[1]);   // rotation
      pt.z = static_cast<double>(polar[2]);   // height
      polar_pub_->publish(pt);
    }

    // ── Pump status ────────────────────────────────────────────────────────
    const int pump = swift_->get_pump_status();
    if (pump >= 0)
    {
      std_msgs::msg::Int8 msg;
      msg.data = static_cast<int8_t>(pump);
      pump_status_pub_->publish(msg);
    }

    // ── Gripper status ─────────────────────────────────────────────────────
    const int gripper = swift_->get_gripper_catch();
    if (gripper >= 0)
    {
      std_msgs::msg::Int8 msg;
      msg.data = static_cast<int8_t>(gripper);
      gripper_status_pub_->publish(msg);
    }

    // ── Is moving ─────────────────────────────────────────────────────────
    const int moving = swift_->get_is_moving();
    if (moving >= 0)
    {
      std_msgs::msg::Bool msg;
      msg.data = (moving == 1);
      is_moving_pub_->publish(msg);
    }

    // ── Mode ───────────────────────────────────────────────────────────────
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
    // Execute in a detached thread — do not block the action server
    std::thread(
      std::bind(&SwiftProHardware::execute_move_arm, this, goal_handle)
    ).detach();
  }

  void execute_move_arm(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
  {
    const auto goal     = goal_handle->get_goal();
    auto       feedback = std::make_shared<MoveArm::Feedback>();
    auto       result   = std::make_shared<MoveArm::Result>();

    // Fire the move asynchronously — do not block here.
    // The uarm library returns -1 immediately in async mode regardless of outcome;
    // completion is detected by polling get_is_moving().
    swift_->set_position(
      goal->x, goal->y, goal->z,
      static_cast<long>(goal->speed),
      false,   // relative
      false);  // wait

    // Poll at 10 Hz — publish feedback, check for cancel
    constexpr auto poll_interval = std::chrono::milliseconds(100);
    constexpr int  max_polls     = 300;  // 30 second timeout

    for (int i = 0; i < max_polls; ++i)
    {
      std::this_thread::sleep_for(poll_interval);

      // ── Cancel check ───────────────────────────────────────────────────
      if (goal_handle->is_canceling())
      {
        // Best-effort stop: flush queue, then command current position
        swift_->flush_cmd();
        {
          std::lock_guard<std::mutex> lock(position_mutex_);
          if (current_position_.size() >= 3)
          {
            swift_->set_position(
              current_position_[0], current_position_[1], current_position_[2],
              static_cast<long>(goal->speed),
              false, false);
          }
        }

        result->success    = false;
        result->error_code = -3;
        result->message    = "Cancelled";
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "move_arm: cancelled");
        return;
      }

      // ── Publish feedback ───────────────────────────────────────────────
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

      // ── Completion check ───────────────────────────────────────────────
      const int moving = swift_->get_is_moving();
      if (moving == 0)
      {
        break;
      }
    }

    // ── Verify final position ──────────────────────────────────────────────
    // Read actual position and compare against goal within tolerance.
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

    const float dx = std::abs(actual_x - goal->x);
    const float dy = std::abs(actual_y - goal->y);
    const float dz = std::abs(actual_z - goal->z);
    const float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    const bool  in_tolerance = (dist <= static_cast<float>(move_tolerance_mm_));

    if (goal_handle->is_active())
    {
      result->success    = in_tolerance;
      result->error_code = in_tolerance ? 0 : -2;
      if (in_tolerance)
      {
        result->message = "OK";
      }
      else
      {
        char buf[128];
        std::snprintf(buf, sizeof(buf),
          "Position tolerance exceeded: dist=%.1fmm (tol=%.1fmm) "
          "actual=(%.1f,%.1f,%.1f)",
          dist, move_tolerance_mm_, actual_x, actual_y, actual_z);
        result->message = buf;
      }

      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(),
                  "move_arm: %s dist=%.1fmm actual=(%.1f,%.1f,%.1f)",
                  in_tolerance ? "succeeded" : "failed",
                  dist, actual_x, actual_y, actual_z);
    }
  }

  // ---------------------------------------------------------------------------
  // Service handlers — motion
  // ---------------------------------------------------------------------------

  void handle_set_polar(
    const std::shared_ptr<swiftpro_resources::srv::SetPolar::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetPolar::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result = swift_->set_polar(
      request->stretch, request->rotation, request->height,
      static_cast<long>(request->speed),
      request->relative,
      request->wait);

    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(),
                "set_polar: stretch=%.1f rotation=%.1f height=%.1f result=%d",
                request->stretch, request->rotation, request->height, result);
  }

  void handle_set_wrist(
    const std::shared_ptr<swiftpro_resources::srv::SetWrist::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetWrist::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result = swift_->set_wrist(
      request->angle,
      static_cast<long>(request->speed));

    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "set_wrist: angle=%.1f result=%d",
                request->angle, result);
  }

  void handle_set_servo_angle(
    const std::shared_ptr<swiftpro_resources::srv::SetServoAngle::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetServoAngle::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result = swift_->set_servo_angle(
      request->servo_id,
      request->angle,
      static_cast<long>(request->speed),
      request->wait);

    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(),
                "set_servo_angle: servo=%d angle=%.1f result=%d",
                request->servo_id, request->angle, result);
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

    const float x = (request->x > 0.0f)  ? request->x : 200.0f;
    const float y = (request->y != 0.0f) ? request->y : 0.0f;
    const float z = (request->z > 0.0f)  ? request->z : 150.0f;

    swift_->reset(static_cast<long>(request->speed), 10.0f, x, y, z);

    response->success = true;
    RCLCPP_INFO(get_logger(),
                "reset: complete — x=%.1f y=%.1f z=%.1f", x, y, z);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — end effectors
  // ---------------------------------------------------------------------------

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

  void handle_set_buzzer(
    const std::shared_ptr<swiftpro_resources::srv::SetBuzzer::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetBuzzer::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result     = swift_->set_buzzer(request->frequency, request->duration);
    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "set_buzzer: freq=%d dur=%.1f result=%d",
                request->frequency, request->duration, result);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — servo management
  // ---------------------------------------------------------------------------

  void handle_set_servo_attach(
    const std::shared_ptr<swiftpro_resources::srv::SetServoAttach::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetServoAttach::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result = request->attach
      ? swift_->set_servo_attach(request->servo_id)
      : swift_->set_servo_detach(request->servo_id);

    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "set_servo_attach: servo=%d attach=%s result=%d",
                request->servo_id, request->attach ? "true" : "false", result);
  }

  void handle_get_servo_attach(
    const std::shared_ptr<swiftpro_resources::srv::GetServoAttach::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::GetServoAttach::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->attached = false;
      response->success  = false;
      return;
    }

    const int result      = swift_->get_servo_attach(request->servo_id);
    response->success     = (result >= 0);
    response->attached    = (result == 0);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — configuration
  // ---------------------------------------------------------------------------

  void handle_set_mode(
    const std::shared_ptr<swiftpro_resources::srv::SetMode::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetMode::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result     = swift_->set_mode(request->mode);
    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "set_mode: mode=%d result=%d",
                request->mode, result);
  }

  void handle_set_acceleration(
    const std::shared_ptr<swiftpro_resources::srv::SetAcceleration::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetAcceleration::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result     = swift_->set_acceleration(request->acc);
    response->success    = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "set_acceleration: acc=%.2f result=%d",
                request->acc, result);
  }

  // ---------------------------------------------------------------------------
  // Service handlers — GPIO
  // ---------------------------------------------------------------------------

  void handle_set_digital_output(
    const std::shared_ptr<swiftpro_resources::srv::SetDigitalOutput::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetDigitalOutput::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result     = swift_->set_digital_output(request->pin, request->value);
    response->success    = (result == 0);
    response->error_code = result;
  }

  void handle_set_digital_direction(
    const std::shared_ptr<swiftpro_resources::srv::SetDigitalDirection::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::SetDigitalDirection::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success    = false;
      response->error_code = -1;
      return;
    }

    const int result     = swift_->set_digital_direction(request->pin, request->value);
    response->success    = (result == 0);
    response->error_code = result;
  }

  void handle_get_digital(
    const std::shared_ptr<swiftpro_resources::srv::GetDigital::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::GetDigital::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->value   = -1;
      response->success = false;
      return;
    }

    const int result  = swift_->get_digital(request->pin);
    response->value   = result;
    response->success = (result >= 0);
  }

  void handle_get_analog(
    const std::shared_ptr<swiftpro_resources::srv::GetAnalog::Request>  request,
          std::shared_ptr<swiftpro_resources::srv::GetAnalog::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->value   = -1;
      response->success = false;
      return;
    }

    const int result  = swift_->get_analog(request->pin);
    response->value   = result;
    response->success = (result >= 0);
  }

  // ---------------------------------------------------------------------------
  // Member variables
  // ---------------------------------------------------------------------------

  // uarm driver
  std::unique_ptr<uarm::Swift> swift_;
  std::string                  port_;
  int                          baudrate_;
  double                       position_report_interval_;
  double                       move_tolerance_mm_;

  // Position state — written from uarm callback thread, read from timer/action threads
  std::mutex         position_mutex_;
  std::vector<float> current_position_;

  // Static trampoline target — one instance only (uarm API limitation)
  static SwiftProHardware* instance_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr     position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr     polar_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           pump_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           gripper_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           is_moving_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           limit_switch_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           mode_pub_;

  // Action server
  rclcpp_action::Server<MoveArm>::SharedPtr move_arm_server_;

  // Services
  rclcpp::Service<swiftpro_resources::srv::SetPump>::SharedPtr             set_pump_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetGripper>::SharedPtr          set_gripper_srv_;
  rclcpp::Service<swiftpro_resources::srv::Reset>::SharedPtr               reset_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetWrist>::SharedPtr            set_wrist_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetServoAttach>::SharedPtr      set_servo_attach_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetMode>::SharedPtr             set_mode_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetPolar>::SharedPtr            set_polar_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetServoAngle>::SharedPtr       set_servo_angle_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetBuzzer>::SharedPtr           set_buzzer_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetAcceleration>::SharedPtr     set_acceleration_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetDigitalOutput>::SharedPtr    set_digital_output_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetDigitalDirection>::SharedPtr set_digital_direction_srv_;
  rclcpp::Service<swiftpro_resources::srv::GetDigital>::SharedPtr          get_digital_srv_;
  rclcpp::Service<swiftpro_resources::srv::GetAnalog>::SharedPtr           get_analog_srv_;
  rclcpp::Service<swiftpro_resources::srv::GetServoAttach>::SharedPtr      get_servo_attach_srv_;

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
