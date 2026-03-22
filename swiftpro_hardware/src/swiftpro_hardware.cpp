// Copyright 2025 Jack Sidman Smith
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Standard library
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// Third-party
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

// uarm
#include "uarm/uarm.h"

// Project messages
#include "swiftpro_resources/srv/move_to.hpp"
#include "swiftpro_resources/srv/set_pump.hpp"
#include "swiftpro_resources/srv/set_gripper.hpp"
#include "swiftpro_resources/srv/reset.hpp"

class SwiftProHardware : public rclcpp::Node
{
public:
  SwiftProHardware()
  : Node("swiftpro_hardware")
  , swift_(nullptr)
  {
    // Parameters
    declare_parameter<std::string>("port", "/dev/ttyACM0");
    declare_parameter<int>("baudrate", 115200);
    declare_parameter<double>("position_report_interval", 0.1);
    declare_parameter<double>("joint_state_publish_rate", 10.0);

    port_ = get_parameter("port").as_string();
    baudrate_ = get_parameter("baudrate").as_int();
    position_report_interval_ = get_parameter("position_report_interval").as_double();
    double publish_rate = get_parameter("joint_state_publish_rate").as_double();

    // Publishers
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      "joint_states", 10);
    position_pub_ = create_publisher<geometry_msgs::msg::Point>(
      "swiftpro/position", 10);
    pump_status_pub_ = create_publisher<std_msgs::msg::Bool>(
      "swiftpro/pump_status", 10);
    connected_pub_ = create_publisher<std_msgs::msg::Bool>(
      "swiftpro/connected", 10);

    // Services
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

    // Connect to arm
    connect();

    // Timers
    auto publish_period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
      std::bind(&SwiftProHardware::publish_state, this));

    // Connection watchdog — checks every 5 seconds
    watchdog_timer_ = create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&SwiftProHardware::check_connection, this));

    RCLCPP_INFO(get_logger(), "SwiftPro Hardware node started on port %s", port_.c_str());
  }

  ~SwiftProHardware()
  {
    if (swift_ && swift_->connected)
    {
      swift_->disconnect();
    }
  }

private:
  // -----------------------------------------------------------------------------
  // Connection management
  // -----------------------------------------------------------------------------

  void connect()
  {
    try
    {
      swift_ = std::make_unique<uarm::Swift>(port_, static_cast<uint32_t>(baudrate_));

      if (!swift_->connected)
      {
        RCLCPP_ERROR(get_logger(), "Failed to connect to UArm Swift Pro on %s", port_.c_str());
        swift_.reset();
        return;
      }

      // Register position report callback
      swift_->set_report_position(static_cast<float>(position_report_interval_));
      // Register position report handler — uarm API requires raw function pointer
      swift_->register_report_position_callback(SwiftProHardware::position_callback);

      RCLCPP_INFO(get_logger(), "Connected to UArm Swift Pro on %s", port_.c_str());

      // Log device info
      std::string* info = swift_->get_device_info();
      if (info)
      {
        RCLCPP_INFO(get_logger(), "Device type: %s", info[0].c_str());
        RCLCPP_INFO(get_logger(), "Hardware version: %s", info[1].c_str());
        RCLCPP_INFO(get_logger(), "Firmware version: %s", info[2].c_str());
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Exception connecting to arm: %s", e.what());
      swift_.reset();
    }
  }

  void check_connection()
  {
    bool connected = swift_ && swift_->connected;
    if (!connected)
    {
      RCLCPP_WARN(get_logger(), "Arm disconnected, attempting reconnect...");
      connect();
    }

    // Always publish connected state
    std_msgs::msg::Bool msg;
    msg.data = connected;
    connected_pub_->publish(msg);
  }

  // -----------------------------------------------------------------------------
  // Position  callback "trampoline"
  // static function that "bounces" a call from a C-style callback 
  // ( which can't capture this ) into a member function on a specific instance. 
  // -----------------------------------------------------------------------------
  static void position_callback(std::vector<float> position)
  {
    if (instance_)
    {
      instance_->on_position_report(position);
    }
  }
  
  
  // -----------------------------------------------------------------------------
  // Position report callback (called from uarm library thread)
  // -----------------------------------------------------------------------------

  void on_position_report(const std::vector<float>& position)
  {
    if (position.size() < 3)
    {
      return;
    }

    std::lock_guard<std::mutex> lock(position_mutex_);
    current_position_ = position;
  }

  // -----------------------------------------------------------------------------
  // State publishing
  // -----------------------------------------------------------------------------

  void publish_state()
  {
    if (!swift_ || !swift_->connected)
    {
      return;
    }

    // Publish XYZ position
    {
      std::lock_guard<std::mutex> lock(position_mutex_);
      if (!current_position_.empty())
      {
        geometry_msgs::msg::Point pos_msg;
        pos_msg.x = current_position_[0];
        pos_msg.y = current_position_[1];
        pos_msg.z = current_position_[2];
        position_pub_->publish(pos_msg);
      }
    }

    // Publish joint states from servo angles
    auto angles = swift_->get_servo_angle();
    if (angles.size() >= 3)
    {
      sensor_msgs::msg::JointState js_msg;
      js_msg.header.stamp = now();
      js_msg.name = {"Joint1", "Joint2", "Joint3"};
      js_msg.position = {
        angles[0] * M_PI / 180.0,
        angles[1] * M_PI / 180.0,
        angles[2] * M_PI / 180.0
      };
      joint_state_pub_->publish(js_msg);
    }

    // Publish pump status
    int pump = swift_->get_pump_status();
    if (pump >= 0)
    {
      std_msgs::msg::Bool pump_msg;
      pump_msg.data = (pump > 0);
      pump_status_pub_->publish(pump_msg);
    }
  }

  // -----------------------------------------------------------------------------
  // Service handlers
  // -----------------------------------------------------------------------------

  void handle_move_to(
    const std::shared_ptr<swiftpro_resources::srv::MoveTo::Request> request,
    std::shared_ptr<swiftpro_resources::srv::MoveTo::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success = false;
      response->error_code = -1;
      response->message = "Arm not connected";
      return;
    }

    RCLCPP_INFO(get_logger(), "MoveTo: x=%.1f y=%.1f z=%.1f speed=%.0f",
                request->x, request->y, request->z, request->speed);

    int result = swift_->set_position(
      request->x, request->y, request->z,
      static_cast<long>(request->speed),
      false,
      request->wait);

    response->success = (result == 0);
    response->error_code = result;
    response->message = (result == 0) ? "OK" : "Movement failed";
  }

  void handle_set_pump(
    const std::shared_ptr<swiftpro_resources::srv::SetPump::Request> request,
    std::shared_ptr<swiftpro_resources::srv::SetPump::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success = false;
      response->error_code = -1;
      return;
    }

    int result = swift_->set_pump(request->on);
    response->success = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "Pump %s: result=%d",
                request->on ? "ON" : "OFF", result);
  }

  void handle_set_gripper(
    const std::shared_ptr<swiftpro_resources::srv::SetGripper::Request> request,
    std::shared_ptr<swiftpro_resources::srv::SetGripper::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success = false;
      response->error_code = -1;
      return;
    }

    int result = swift_->set_gripper(request->catch_object);
    response->success = (result == 0);
    response->error_code = result;

    RCLCPP_INFO(get_logger(), "Gripper %s: result=%d",
                request->catch_object ? "CATCH" : "RELEASE", result);
  }

  void handle_reset(
    const std::shared_ptr<swiftpro_resources::srv::Reset::Request> request,
    std::shared_ptr<swiftpro_resources::srv::Reset::Response> response)
  {
    if (!swift_ || !swift_->connected)
    {
      response->success = false;
      return;
    }

    swift_->reset(
      static_cast<long>(request->speed),
      10.0f,
      request->x > 0 ? request->x : 200.0f,
      request->y != 0 ? request->y : 0.0f,
      request->z > 0 ? request->z : 150.0f);

    response->success = true;
    RCLCPP_INFO(get_logger(), "Reset complete");
  }

  // -----------------------------------------------------------------------------
  // Member variables
  // -----------------------------------------------------------------------------

  // uarm
  std::unique_ptr<uarm::Swift> swift_;
  std::string port_;
  int baudrate_;
  double position_report_interval_;

  // Position state
  std::mutex position_mutex_;
  std::vector<float> current_position_;

  // Static instance pointer for uarm callbacks
  static SwiftProHardware* instance_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pump_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;

  // Services
  rclcpp::Service<swiftpro_resources::srv::MoveTo>::SharedPtr move_to_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetPump>::SharedPtr set_pump_srv_;
  rclcpp::Service<swiftpro_resources::srv::SetGripper>::SharedPtr set_gripper_srv_;
  rclcpp::Service<swiftpro_resources::srv::Reset>::SharedPtr reset_srv_;

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
