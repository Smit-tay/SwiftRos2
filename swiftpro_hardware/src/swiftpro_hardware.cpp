// swiftpro_hardware.cpp
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Hardware interface node for the UArm Swift Pro robotic arm.
//
// Connects to the arm via USB serial (libswiftpro) and provides a complete
// ROS2 surface: event-driven joint/position publishing, latched connection
// and power state, an action server for long Cartesian moves with feedback
// and cancellation, and services for everything libswiftpro exposes.
//
//   Publishers (7):
//     /joint_states              sensor_msgs/JointState  base, left, right
//     /swiftpro/position         geometry_msgs/Point     (x, y, z) mm
//     /swiftpro/pump_status      std_msgs/Int8           0=off, 1=working, 2=holding
//     /swiftpro/gripper_status   std_msgs/Int8           0=released, 1=working, 2=holding
//     /swiftpro/connected        std_msgs/Bool           latched
//     /swiftpro/power            std_msgs/Bool           latched
//     /swiftpro/limit_switch     std_msgs/Bool
//     /swiftpro/motion_complete  std_msgs/Empty          fires on @9 event
//
//   Action (1):
//     /swiftpro/move_arm         swiftpro_resources/action/MoveArm
//       Goal x,y,z (mm), speed (mm/min). Feedback at 10 Hz. Cancel = motion_reset.
//
//   Services: see definitions in swiftpro_resources/messaging/srv/.
//
// Threading: MultiThreadedExecutor with one reentrant callback group for all
// services and the action server, so libswiftpro _sync calls can run in
// parallel from independent clients.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int8.hpp>

#include <swiftpro/swiftpro.h>

#include "swiftpro_resources/action/move_arm.hpp"
#include "swiftpro_resources/srv/angles_to_coord.hpp"
#include "swiftpro_resources/srv/coord_to_angles.hpp"
#include "swiftpro_resources/srv/get_analog.hpp"
#include "swiftpro_resources/srv/get_digital.hpp"
#include "swiftpro_resources/srv/get_encoder_status.hpp"
#include "swiftpro_resources/srv/get_servo_attach.hpp"
#include "swiftpro_resources/srv/is_reachable.hpp"
#include "swiftpro_resources/srv/motion_reset.hpp"
#include "swiftpro_resources/srv/move_to.hpp"
#include "swiftpro_resources/srv/pause_motion.hpp"
#include "swiftpro_resources/srv/reset.hpp"
#include "swiftpro_resources/srv/resume_motion.hpp"
#include "swiftpro_resources/srv/send_raw.hpp"
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

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

static const std::vector<std::string> JOINT_NAMES = { "base", "left", "right" };

static constexpr double DEG_TO_RAD        = M_PI / 180.0;
static constexpr int    CONNECT_SETTLE_MS = 2000;   // DTR reset boot delay

// Pull libswiftpro result aliases into the TU scope.
using swiftpro::VoidResult;

// -----------------------------------------------------------------------------

class SwiftProHardware : public rclcpp::Node
{
public:
    using MoveArm           = swiftpro_resources::action::MoveArm;
    using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

    SwiftProHardware()
    : Node("swiftpro_hardware")
    {
        // ── Parameters ────────────────────────────────────────────────────
        port_                     = declare_parameter<std::string>("port",                    "/dev/ttyACM0");
        baudrate_                 = declare_parameter<int>        ("baudrate",                115200);
        position_report_interval_ = declare_parameter<double>     ("position_report_interval", 0.1);
        status_poll_rate_hz_      = declare_parameter<double>     ("status_poll_rate",         1.0);
        watchdog_period_s_        = declare_parameter<double>     ("watchdog_period",          5.0);

        // ── QoS profiles ──────────────────────────────────────────────────
        const rclcpp::QoS latched_qos =
            rclcpp::QoS(1).transient_local().reliable();

        // ── Publishers ────────────────────────────────────────────────────
        joint_state_pub_     = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        position_pub_        = create_publisher<geometry_msgs::msg::Point>   ("swiftpro/position", 10);
        pump_status_pub_     = create_publisher<std_msgs::msg::Int8>         ("swiftpro/pump_status", 10);
        gripper_status_pub_  = create_publisher<std_msgs::msg::Int8>         ("swiftpro/gripper_status", 10);
        connected_pub_       = create_publisher<std_msgs::msg::Bool>         ("swiftpro/connected", latched_qos);
        power_pub_           = create_publisher<std_msgs::msg::Bool>         ("swiftpro/power", latched_qos);
        limit_switch_pub_    = create_publisher<std_msgs::msg::Bool>         ("swiftpro/limit_switch", 10);
        motion_complete_pub_ = create_publisher<std_msgs::msg::Empty>        ("swiftpro/motion_complete", 10);

        // Single reentrant callback group for everything that may call into
        // libswiftpro _sync — services and the action server. Lets a
        // MultiThreadedExecutor run them in parallel from independent clients.
        cb_group_ = create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // ── Action server ─────────────────────────────────────────────────
        move_arm_server_ = rclcpp_action::create_server<MoveArm>(
            this, "swiftpro/move_arm",
            std::bind(&SwiftProHardware::handle_goal,     this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&SwiftProHardware::handle_cancel,   this,
                      std::placeholders::_1),
            std::bind(&SwiftProHardware::handle_accepted, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(),
            cb_group_);

        // ── Services ──────────────────────────────────────────────────────
        // Motion
        move_to_srv_         = make_srv<swiftpro_resources::srv::MoveTo>             ("swiftpro/move_to",          &SwiftProHardware::handle_move_to);
        set_polar_srv_       = make_srv<swiftpro_resources::srv::SetPolar>           ("swiftpro/set_polar",        &SwiftProHardware::handle_set_polar);
        set_servo_angle_srv_ = make_srv<swiftpro_resources::srv::SetServoAngle>      ("swiftpro/set_servo_angle",  &SwiftProHardware::handle_set_servo_angle);
        set_servo_angles_srv_= make_srv<swiftpro_resources::srv::SetServoAngles>     ("swiftpro/set_servo_angles", &SwiftProHardware::handle_set_servo_angles);
        set_wrist_srv_       = make_srv<swiftpro_resources::srv::SetWrist>           ("swiftpro/set_wrist",        &SwiftProHardware::handle_set_wrist);
        pause_motion_srv_    = make_srv<swiftpro_resources::srv::PauseMotion>        ("swiftpro/pause_motion",     &SwiftProHardware::handle_pause_motion);
        resume_motion_srv_   = make_srv<swiftpro_resources::srv::ResumeMotion>       ("swiftpro/resume_motion",    &SwiftProHardware::handle_resume_motion);
        motion_reset_srv_    = make_srv<swiftpro_resources::srv::MotionReset>        ("swiftpro/motion_reset",     &SwiftProHardware::handle_motion_reset);
        reset_srv_           = make_srv<swiftpro_resources::srv::Reset>              ("swiftpro/reset",            &SwiftProHardware::handle_reset);

        // End effectors
        set_pump_srv_        = make_srv<swiftpro_resources::srv::SetPump>            ("swiftpro/set_pump",         &SwiftProHardware::handle_set_pump);
        set_gripper_srv_     = make_srv<swiftpro_resources::srv::SetGripper>         ("swiftpro/set_gripper",      &SwiftProHardware::handle_set_gripper);
        set_buzzer_srv_      = make_srv<swiftpro_resources::srv::SetBuzzer>          ("swiftpro/set_buzzer",       &SwiftProHardware::handle_set_buzzer);

        // Configuration
        set_mode_srv_        = make_srv<swiftpro_resources::srv::SetMode>            ("swiftpro/set_mode",         &SwiftProHardware::handle_set_mode);
        set_acceleration_srv_= make_srv<swiftpro_resources::srv::SetAcceleration>    ("swiftpro/set_acceleration", &SwiftProHardware::handle_set_acceleration);

        // Servo management
        set_servo_attach_srv_= make_srv<swiftpro_resources::srv::SetServoAttach>     ("swiftpro/set_servo_attach", &SwiftProHardware::handle_set_servo_attach);
        get_servo_attach_srv_= make_srv<swiftpro_resources::srv::GetServoAttach>     ("swiftpro/get_servo_attach", &SwiftProHardware::handle_get_servo_attach);
        get_encoder_status_srv_= make_srv<swiftpro_resources::srv::GetEncoderStatus> ("swiftpro/get_encoder_status",&SwiftProHardware::handle_get_encoder_status);

        // GPIO
        set_digital_output_srv_= make_srv<swiftpro_resources::srv::SetDigitalOutput> ("swiftpro/set_digital_output",   &SwiftProHardware::handle_set_digital_output);
        set_digital_direction_srv_= make_srv<swiftpro_resources::srv::SetDigitalDirection>("swiftpro/set_digital_direction",&SwiftProHardware::handle_set_digital_direction);
        get_digital_srv_     = make_srv<swiftpro_resources::srv::GetDigital>         ("swiftpro/get_digital",      &SwiftProHardware::handle_get_digital);
        get_analog_srv_      = make_srv<swiftpro_resources::srv::GetAnalog>          ("swiftpro/get_analog",       &SwiftProHardware::handle_get_analog);

        // Queries (new)
        is_reachable_srv_    = make_srv<swiftpro_resources::srv::IsReachable>        ("swiftpro/is_reachable",     &SwiftProHardware::handle_is_reachable);
        coord_to_angles_srv_ = make_srv<swiftpro_resources::srv::CoordToAngles>      ("swiftpro/coord_to_angles",  &SwiftProHardware::handle_coord_to_angles);
        angles_to_coord_srv_ = make_srv<swiftpro_resources::srv::AnglesToCoord>      ("swiftpro/angles_to_coord",  &SwiftProHardware::handle_angles_to_coord);
        send_raw_srv_        = make_srv<swiftpro_resources::srv::SendRaw>            ("swiftpro/send_raw",         &SwiftProHardware::handle_send_raw);

        RCLCPP_INFO(get_logger(),
                    "SwiftPro hardware node — Jack Sidman Smith — built %s %s",
                    __DATE__, __TIME__);

        connect();

        // ── Periodic timers ───────────────────────────────────────────────
        const auto status_period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / status_poll_rate_hz_));

        status_timer_ = create_wall_timer(
            status_period,
            std::bind(&SwiftProHardware::poll_status, this),
            cb_group_);

        watchdog_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(watchdog_period_s_)),
            std::bind(&SwiftProHardware::watchdog, this),
            cb_group_);

        RCLCPP_INFO(get_logger(),
                    "swiftpro_hardware ready — port=%s baud=%d status=%.1fHz watchdog=%.1fs",
                    port_.c_str(), baudrate_,
                    status_poll_rate_hz_, watchdog_period_s_);
    }

    ~SwiftProHardware() override
    {
        if (swift_.is_connected()) { swift_.disconnect(); }
    }

private:
    // ─────────────────────────────────────────────────────────────────────
    // Service factory — keeps the constructor compact.
    // ─────────────────────────────────────────────────────────────────────
    template<typename SrvT, typename Fn>
    typename rclcpp::Service<SrvT>::SharedPtr
    make_srv(const std::string& name, Fn handler)
    {
        return create_service<SrvT>(
            name,
            [this, handler](
                const std::shared_ptr<typename SrvT::Request>  req,
                      std::shared_ptr<typename SrvT::Response> res)
            { (this->*handler)(req, res); },
            rclcpp::ServicesQoS(),
            cb_group_);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Connection lifecycle
    // ─────────────────────────────────────────────────────────────────────
    void connect()
    {
        if (swift_.is_connected()) { return; }

        RCLCPP_INFO(get_logger(), "connect: opening %s @ %d", port_.c_str(), baudrate_);

        if (!swift_.connect(port_, static_cast<uint32_t>(baudrate_))) {
            RCLCPP_ERROR(get_logger(), "connect: failed to open %s", port_.c_str());
            publish_connected(false);
            return;
        }

        // Allow firmware to complete boot after DTR/RTS reset.
        std::this_thread::sleep_for(std::chrono::milliseconds(CONNECT_SETTLE_MS));

        // Register event callbacks BEFORE enabling reports.
        // All event callbacks run on the receiver thread — they only touch
        // publishers (thread-safe) and atomic/mutex-protected cache.
        swift_.on_position([this](float x, float y, float z, float /*wrist*/) {
            on_position_event(x, y, z);
        });
        swift_.on_joint([this](float b, float l, float r) {
            on_joint_event(b, l, r);
        });
        swift_.on_motion_complete([this]() {
            on_motion_complete_event();
        });
        swift_.on_limit_switch([this](bool state) {
            on_limit_switch_event(state);
        });
        swift_.on_power([this](bool on) {
            on_power_event(on);
        });

        // Enable @3 periodic position/joint reports.
        if (auto r = swift_.set_report_interval_sync(
                static_cast<float>(position_report_interval_));
            !r)
        {
            RCLCPP_WARN(get_logger(), "set_report_interval: %s", r.error().c_str());
        }

        // Enable @9 motion-complete events.
        if (auto r = swift_.set_motion_report_sync(true); !r) {
            RCLCPP_WARN(get_logger(), "set_motion_report: %s", r.error().c_str());
        }

        // Device info (best-effort).
        if (auto r = swift_.get_device_info_sync(); r) {
            const auto& info = r.value();
            RCLCPP_INFO(get_logger(), "Device type     : %s", info[0].c_str());
            RCLCPP_INFO(get_logger(), "Hardware version: %s", info[1].c_str());
            RCLCPP_INFO(get_logger(), "Firmware version: %s", info[2].c_str());
            RCLCPP_INFO(get_logger(), "API version     : %s", info[3].c_str());
            RCLCPP_INFO(get_logger(), "Device unique   : %s", info[4].c_str());
        }

        // Encoder health (best-effort).
        if (auto r = swift_.get_encoder_status_sync(); r) {
            if (r.value() == 0) {
                RCLCPP_INFO(get_logger(), "encoders: all healthy");
            } else {
                RCLCPP_WARN(get_logger(),
                            "encoders: fault bitfield=0x%x "
                            "(bit0=base bit1=right bit2=left)", r.value());
            }
        }

        publish_connected(true);
        RCLCPP_INFO(get_logger(), "connect: ready on %s", port_.c_str());
    }

    void watchdog()
    {
        const bool connected = swift_.is_connected();
        publish_connected(connected);

        if (!connected) {
            RCLCPP_WARN(get_logger(), "watchdog: arm disconnected — reconnecting");
            connect();
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Event handlers (run on libswiftpro receiver thread — must be fast)
    // ─────────────────────────────────────────────────────────────────────
    void on_position_event(float x, float y, float z)
    {
        // Cache for action feedback.
        {
            std::lock_guard<std::mutex> lock(position_mutex_);
            cached_x_ = x; cached_y_ = y; cached_z_ = z;
            position_valid_ = true;
        }
        geometry_msgs::msg::Point pt;
        pt.x = x; pt.y = y; pt.z = z;
        position_pub_->publish(pt);
    }

    void on_joint_event(float b, float l, float r)
    {
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name         = JOINT_NAMES;
        js.position     = { b * DEG_TO_RAD, l * DEG_TO_RAD, r * DEG_TO_RAD };
        joint_state_pub_->publish(js);
    }

    void on_motion_complete_event()
    {
        motion_complete_pub_->publish(std_msgs::msg::Empty());
        motion_pending_.store(false);
        motion_cv_.notify_all();
    }

    void on_limit_switch_event(bool state)
    {
        std_msgs::msg::Bool msg; msg.data = state;
        limit_switch_pub_->publish(msg);
    }

    void on_power_event(bool on)
    {
        std_msgs::msg::Bool msg; msg.data = on;
        power_pub_->publish(msg);
    }

    void publish_connected(bool state)
    {
        if (state == last_connected_published_) { return; }
        std_msgs::msg::Bool msg; msg.data = state;
        connected_pub_->publish(msg);
        last_connected_published_ = state;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Periodic poll — pump/gripper status (no event for these)
    // ─────────────────────────────────────────────────────────────────────
    void poll_status()
    {
        if (!swift_.is_connected()) { return; }

        if (auto r = swift_.get_pump_status_sync(); r) {
            std_msgs::msg::Int8 msg;
            msg.data = static_cast<int8_t>(r.value());
            pump_status_pub_->publish(msg);
        }

        if (auto r = swift_.get_gripper_status_sync(); r) {
            std_msgs::msg::Int8 msg;
            msg.data = static_cast<int8_t>(r.value());
            gripper_status_pub_->publish(msg);
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Action server — MoveArm
    // ─────────────────────────────────────────────────────────────────────
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const MoveArm::Goal> goal)
    {
        if (!swift_.is_connected()) {
            RCLCPP_WARN(get_logger(), "move_arm: rejected — arm not connected");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal_active_.exchange(true)) {
            RCLCPP_WARN(get_logger(), "move_arm: rejected — goal already active");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(get_logger(),
                    "move_arm: accepted x=%.1f y=%.1f z=%.1f speed=%.0f",
                    goal->x, goal->y, goal->z, goal->speed);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleMoveArm> /*gh*/)
    {
        RCLCPP_INFO(get_logger(), "move_arm: cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMoveArm> gh)
    {
        std::thread(&SwiftProHardware::execute_move_arm, this, gh).detach();
    }

    void execute_move_arm(const std::shared_ptr<GoalHandleMoveArm> gh)
    {
        const auto goal     = gh->get_goal();
        auto       feedback = std::make_shared<MoveArm::Feedback>();
        auto       result   = std::make_shared<MoveArm::Result>();

        // Arm the motion-complete latch and fire the move asynchronously.
        motion_pending_.store(true);
        swift_.set_position(goal->x, goal->y, goal->z, goal->speed, false,
                            [this](VoidResult r) {
                                if (!r) {
                                    RCLCPP_WARN(get_logger(),
                                                "move_arm: set_position ack: %s",
                                                r.error().c_str());
                                    motion_pending_.store(false);
                                    motion_cv_.notify_all();
                                }
                            });

        // Wait for @9 motion-complete event, publishing feedback at 10 Hz.
        constexpr auto timeout   = std::chrono::seconds(60);
        constexpr auto poll_step = std::chrono::milliseconds(100);
        const auto     deadline  = std::chrono::steady_clock::now() + timeout;

        while (rclcpp::ok()) {
            if (gh->is_canceling()) {
                swift_.motion_reset_sync();
                motion_pending_.store(false);
                result->success    = false;
                result->error_code = -3;
                result->message    = "Cancelled";
                gh->canceled(result);
                RCLCPP_INFO(get_logger(), "move_arm: cancelled");
                goal_active_.store(false);
                return;
            }

            // Publish feedback from cached position.
            {
                std::lock_guard<std::mutex> lock(position_mutex_);
                if (position_valid_) {
                    feedback->current_x = cached_x_;
                    feedback->current_y = cached_y_;
                    feedback->current_z = cached_z_;
                    gh->publish_feedback(feedback);
                }
            }

            // Wait for motion-complete or timeout slice.
            {
                std::unique_lock<std::mutex> lock(motion_mutex_);
                if (motion_cv_.wait_for(lock, poll_step,
                                        [this] { return !motion_pending_.load(); }))
                {
                    break;
                }
            }

            if (std::chrono::steady_clock::now() >= deadline) {
                swift_.motion_reset_sync();
                motion_pending_.store(false);
                result->success    = false;
                result->error_code = -4;
                result->message    = "Timeout";
                gh->abort(result);
                RCLCPP_WARN(get_logger(), "move_arm: timeout");
                goal_active_.store(false);
                return;
            }
        }

        result->success    = true;
        result->error_code = 0;
        result->message    = "OK";
        gh->succeed(result);
        RCLCPP_INFO(get_logger(), "move_arm: succeeded");
        goal_active_.store(false);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Service handlers — motion
    // ─────────────────────────────────────────────────────────────────────
    void handle_move_to(
        const std::shared_ptr<swiftpro_resources::srv::MoveTo::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::MoveTo::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code, res->message)) { return; }
        const auto r = swift_.set_position_sync(req->x, req->y, req->z, req->speed);
        fill_result(r, res->success, res->error_code, res->message);
    }

    void handle_set_polar(
        const std::shared_ptr<swiftpro_resources::srv::SetPolar::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetPolar::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_polar_sync(req->stretch, req->rotation, req->height, req->speed);
        fill_result(r, res->success, res->error_code);
    }

    void handle_set_servo_angle(
        const std::shared_ptr<swiftpro_resources::srv::SetServoAngle::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetServoAngle::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_servo_angle_sync(req->servo_id, req->angle, req->speed);
        fill_result(r, res->success, res->error_code);
    }

    void handle_set_servo_angles(
        const std::shared_ptr<swiftpro_resources::srv::SetServoAngles::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetServoAngles::Response> res)
    {
        if (!swift_.is_connected()) { res->success = false; return; }
        const auto r = swift_.set_joint_angles_sync(req->j1, req->j2, req->j3, req->speed);
        res->success = static_cast<bool>(r);
        if (!r) {
            RCLCPP_WARN(get_logger(), "set_servo_angles: %s", r.error().c_str());
        }
    }

    void handle_set_wrist(
        const std::shared_ptr<swiftpro_resources::srv::SetWrist::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetWrist::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_wrist_sync(req->angle, req->speed);
        fill_result(r, res->success, res->error_code);
    }

    void handle_pause_motion(
        const std::shared_ptr<swiftpro_resources::srv::PauseMotion::Request>  /*req*/,
              std::shared_ptr<swiftpro_resources::srv::PauseMotion::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.pause_motion_sync();
        fill_result(r, res->success, res->error_code);
    }

    void handle_resume_motion(
        const std::shared_ptr<swiftpro_resources::srv::ResumeMotion::Request>  /*req*/,
              std::shared_ptr<swiftpro_resources::srv::ResumeMotion::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.resume_motion_sync();
        fill_result(r, res->success, res->error_code);
    }

    void handle_motion_reset(
        const std::shared_ptr<swiftpro_resources::srv::MotionReset::Request>  /*req*/,
              std::shared_ptr<swiftpro_resources::srv::MotionReset::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.motion_reset_sync();
        fill_result(r, res->success, res->error_code);
    }

    void handle_reset(
        const std::shared_ptr<swiftpro_resources::srv::Reset::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::Reset::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code, res->message)) { return; }
        const auto r = swift_.reset_sync(req->x, req->y, req->z, req->speed);
        fill_result(r, res->success, res->error_code, res->message);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Service handlers — end effectors
    // ─────────────────────────────────────────────────────────────────────
    void handle_set_pump(
        const std::shared_ptr<swiftpro_resources::srv::SetPump::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetPump::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_pump_sync(req->on);
        fill_result(r, res->success, res->error_code);
    }

    void handle_set_gripper(
        const std::shared_ptr<swiftpro_resources::srv::SetGripper::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetGripper::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_gripper_sync(req->catch_object);
        fill_result(r, res->success, res->error_code);
    }

    void handle_set_buzzer(
        const std::shared_ptr<swiftpro_resources::srv::SetBuzzer::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetBuzzer::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_buzzer_sync(req->frequency, req->duration);
        fill_result(r, res->success, res->error_code);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Service handlers — configuration
    // ─────────────────────────────────────────────────────────────────────
    void handle_set_mode(
        const std::shared_ptr<swiftpro_resources::srv::SetMode::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetMode::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_mode_sync(req->mode);
        fill_result(r, res->success, res->error_code);
    }

    void handle_set_acceleration(
        const std::shared_ptr<swiftpro_resources::srv::SetAcceleration::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetAcceleration::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_acceleration_sync(req->acc);
        fill_result(r, res->success, res->error_code);
    }

    // ─────────────────────────────────────────────────────────────────────
    // Service handlers — servo management
    // ─────────────────────────────────────────────────────────────────────
    void handle_set_servo_attach(
        const std::shared_ptr<swiftpro_resources::srv::SetServoAttach::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetServoAttach::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = req->attach
            ? swift_.set_servo_attach_sync(req->servo_id)
            : swift_.set_servo_detach_sync(req->servo_id);
        fill_result(r, res->success, res->error_code);
    }

    void handle_get_servo_attach(
        const std::shared_ptr<swiftpro_resources::srv::GetServoAttach::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::GetServoAttach::Response> res)
    {
        if (!swift_.is_connected()) {
            res->attached = false; res->success = false; return;
        }
        const auto r = swift_.get_servo_attach_sync(req->servo_id);
        res->success  = static_cast<bool>(r);
        res->attached = (r && r.value() == 1);
        if (!r) {
            RCLCPP_WARN(get_logger(), "get_servo_attach: %s", r.error().c_str());
        }
    }

    void handle_get_encoder_status(
        const std::shared_ptr<swiftpro_resources::srv::GetEncoderStatus::Request>  /*req*/,
              std::shared_ptr<swiftpro_resources::srv::GetEncoderStatus::Response> res)
    {
        if (!swift_.is_connected()) {
            res->status = -1; res->success = false; return;
        }
        const auto r = swift_.get_encoder_status_sync();
        res->success = static_cast<bool>(r);
        res->status  = r ? r.value() : -1;
        if (r && r.value() > 0) {
            RCLCPP_WARN(get_logger(),
                        "get_encoder_status: fault bitfield=0x%x "
                        "(bit0=base bit1=right bit2=left)", r.value());
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // Service handlers — GPIO
    // ─────────────────────────────────────────────────────────────────────
    void handle_set_digital_output(
        const std::shared_ptr<swiftpro_resources::srv::SetDigitalOutput::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetDigitalOutput::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_digital_output_sync(req->pin, req->value);
        fill_result(r, res->success, res->error_code);
    }

    void handle_set_digital_direction(
        const std::shared_ptr<swiftpro_resources::srv::SetDigitalDirection::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SetDigitalDirection::Response> res)
    {
        if (!ensure_connected(res->success, res->error_code)) { return; }
        const auto r = swift_.set_digital_direction_sync(req->pin, req->value);
        fill_result(r, res->success, res->error_code);
    }

    void handle_get_digital(
        const std::shared_ptr<swiftpro_resources::srv::GetDigital::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::GetDigital::Response> res)
    {
        if (!swift_.is_connected()) {
            res->value = -1; res->success = false; return;
        }
        const auto r = swift_.get_digital_sync(req->pin);
        res->success = static_cast<bool>(r);
        res->value   = r ? r.value() : -1;
    }

    void handle_get_analog(
        const std::shared_ptr<swiftpro_resources::srv::GetAnalog::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::GetAnalog::Response> res)
    {
        if (!swift_.is_connected()) {
            res->value = -1; res->success = false; return;
        }
        const auto r = swift_.get_analog_sync(req->pin);
        res->success = static_cast<bool>(r);
        res->value   = r ? r.value() : -1;
    }

    // ─────────────────────────────────────────────────────────────────────
    // Service handlers — queries (new)
    // ─────────────────────────────────────────────────────────────────────
    void handle_is_reachable(
        const std::shared_ptr<swiftpro_resources::srv::IsReachable::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::IsReachable::Response> res)
    {
        if (!swift_.is_connected()) {
            res->reachable = false; res->success = false; res->error_code = -1; return;
        }
        const auto r = swift_.is_reachable_sync(req->x, req->y, req->z);
        res->success   = static_cast<bool>(r);
        res->reachable = (r && r.value() == 1);
        res->error_code = r ? 0 : -2;
        if (!r) {
            RCLCPP_WARN(get_logger(), "is_reachable: %s", r.error().c_str());
        }
    }

    void handle_coord_to_angles(
        const std::shared_ptr<swiftpro_resources::srv::CoordToAngles::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::CoordToAngles::Response> res)
    {
        if (!swift_.is_connected()) {
            res->success = false; res->error_code = -1; return;
        }
        const auto r = swift_.coord_to_angles_sync(req->x, req->y, req->z);
        if (r && r.value().size() >= 3) {
            res->base  = r.value()[0];
            res->left  = r.value()[1];
            res->right = r.value()[2];
            res->success = true; res->error_code = 0;
        } else {
            res->success = false; res->error_code = -2;
            if (!r) {
                RCLCPP_WARN(get_logger(), "coord_to_angles: %s", r.error().c_str());
            }
        }
    }

    void handle_angles_to_coord(
        const std::shared_ptr<swiftpro_resources::srv::AnglesToCoord::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::AnglesToCoord::Response> res)
    {
        if (!swift_.is_connected()) {
            res->success = false; res->error_code = -1; return;
        }
        const auto r = swift_.angles_to_coord_sync(req->base, req->left, req->right);
        if (r && r.value().size() >= 3) {
            res->x = r.value()[0];
            res->y = r.value()[1];
            res->z = r.value()[2];
            res->success = true; res->error_code = 0;
        } else {
            res->success = false; res->error_code = -2;
            if (!r) {
                RCLCPP_WARN(get_logger(), "angles_to_coord: %s", r.error().c_str());
            }
        }
    }

    void handle_send_raw(
        const std::shared_ptr<swiftpro_resources::srv::SendRaw::Request>  req,
              std::shared_ptr<swiftpro_resources::srv::SendRaw::Response> res)
    {
        if (!swift_.is_connected()) {
            res->success = false; res->error_code = -1;
            res->response = ""; return;
        }
        const float timeout = (req->timeout > 0.0f) ? req->timeout : 5.0f;
        const auto r = swift_.send_raw_sync(req->command, timeout);
        // send_raw_sync is VoidResult — there's no captured response string in
        // the public API. Echo the command status as the response payload for
        // shell consumers, and surface the error string on failure.
        res->success    = static_cast<bool>(r);
        res->error_code = r ? 0 : -2;
        res->response   = r ? "ok" : r.error();
    }

    // ─────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────
    bool ensure_connected(bool& success_out, int32_t& err_out)
    {
        if (swift_.is_connected()) { return true; }
        success_out = false; err_out = -1;
        return false;
    }

    bool ensure_connected(bool& success_out, int32_t& err_out, std::string& msg_out)
    {
        if (swift_.is_connected()) { return true; }
        success_out = false; err_out = -1; msg_out = "Not connected";
        return false;
    }

    void fill_result(const VoidResult& r, bool& success_out, int32_t& err_out)
    {
        success_out = static_cast<bool>(r);
        err_out     = r ? 0 : -2;
        if (!r) {
            RCLCPP_WARN(get_logger(), "command failed: %s", r.error().c_str());
        }
    }

    void fill_result(const VoidResult& r, bool& success_out, int32_t& err_out,
                     std::string& msg_out)
    {
        success_out = static_cast<bool>(r);
        err_out     = r ? 0 : -2;
        msg_out     = r ? "OK" : r.error();
        if (!r) {
            RCLCPP_WARN(get_logger(), "command failed: %s", r.error().c_str());
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    // State
    // ─────────────────────────────────────────────────────────────────────
    swiftpro::Swift swift_;

    std::string port_;
    int         baudrate_{115200};
    double      position_report_interval_{0.1};
    double      status_poll_rate_hz_{1.0};
    double      watchdog_period_s_{5.0};

    // Latched-publish state
    bool last_connected_published_{false};

    // Cached position for action feedback (written on receiver thread)
    std::mutex position_mutex_;
    float      cached_x_{0.0f};
    float      cached_y_{0.0f};
    float      cached_z_{0.0f};
    bool       position_valid_{false};

    // Motion-complete latch
    std::mutex              motion_mutex_;
    std::condition_variable motion_cv_;
    std::atomic<bool>       motion_pending_{false};

    // Action goal arbitration
    std::atomic<bool> goal_active_{false};

    // Callback group — all services + action server
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr     position_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           pump_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr           gripper_status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           connected_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           power_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr           limit_switch_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr          motion_complete_pub_;

    // Action server
    rclcpp_action::Server<MoveArm>::SharedPtr move_arm_server_;

    // Services
    rclcpp::Service<swiftpro_resources::srv::MoveTo>::SharedPtr             move_to_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetPolar>::SharedPtr           set_polar_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetServoAngle>::SharedPtr      set_servo_angle_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetServoAngles>::SharedPtr     set_servo_angles_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetWrist>::SharedPtr           set_wrist_srv_;
    rclcpp::Service<swiftpro_resources::srv::PauseMotion>::SharedPtr        pause_motion_srv_;
    rclcpp::Service<swiftpro_resources::srv::ResumeMotion>::SharedPtr       resume_motion_srv_;
    rclcpp::Service<swiftpro_resources::srv::MotionReset>::SharedPtr        motion_reset_srv_;
    rclcpp::Service<swiftpro_resources::srv::Reset>::SharedPtr              reset_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetPump>::SharedPtr            set_pump_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetGripper>::SharedPtr         set_gripper_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetBuzzer>::SharedPtr          set_buzzer_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetMode>::SharedPtr            set_mode_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetAcceleration>::SharedPtr    set_acceleration_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetServoAttach>::SharedPtr     set_servo_attach_srv_;
    rclcpp::Service<swiftpro_resources::srv::GetServoAttach>::SharedPtr     get_servo_attach_srv_;
    rclcpp::Service<swiftpro_resources::srv::GetEncoderStatus>::SharedPtr   get_encoder_status_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetDigitalOutput>::SharedPtr   set_digital_output_srv_;
    rclcpp::Service<swiftpro_resources::srv::SetDigitalDirection>::SharedPtr set_digital_direction_srv_;
    rclcpp::Service<swiftpro_resources::srv::GetDigital>::SharedPtr         get_digital_srv_;
    rclcpp::Service<swiftpro_resources::srv::GetAnalog>::SharedPtr          get_analog_srv_;
    rclcpp::Service<swiftpro_resources::srv::IsReachable>::SharedPtr        is_reachable_srv_;
    rclcpp::Service<swiftpro_resources::srv::CoordToAngles>::SharedPtr      coord_to_angles_srv_;
    rclcpp::Service<swiftpro_resources::srv::AnglesToCoord>::SharedPtr      angles_to_coord_srv_;
    rclcpp::Service<swiftpro_resources::srv::SendRaw>::SharedPtr            send_raw_srv_;

    // Timers
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

// -----------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SwiftProHardware>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
