/*
* \file uarm/uarm.h
# Software License Agreement (MIT License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>
*/
#ifndef UARM_H_
#define UARM_H_

#include <iostream>
#include <functional>
#include <condition_variable>
#include <thread>
#include <mutex>
#include "serial/serial.h"
#include "uarm/timer.h"
#include "uarm/utils.h"
#include <cstddef>

#define MAX_CNT 10000
const float default_timeout_2 = 2;
const float default_timeout_10 = 10;
const long default_speed = 10000;

namespace uarm {
using serial::Timeout;
using serial::PortInfo;
using serial::list_ports;
class Swift {
private:
    // Thread r_thread;
    // Mutex mutex;
    std::thread r_thread;
    // std::mutex mutex_;
    // std::condition_variable cond_;
    int cmd_pend_size;

public:
    /*!
    * @param port: a std::string containing the address of the serial port,
    which would be something like 'COM1' on Windows and '/dev/ttyACM0'
    *        on Linux.
    * @param baudrate: an unsigned 32-bit integer that represents the baudrate
    * @param timeout: A Timeout struct that defines the timeout
    */
    Swift(const std::string &port = "",
        uint32_t baudrate = 115200,
        Timeout timeout = Timeout::simpleTimeout(1000),
        int cmd_pend_size = 2,
        bool do_not_open = false);
    virtual ~Swift();

public:
    std::mutex mutex;
    std::condition_variable cmd_cond;
    serial::Serial ser;
    Timer timer;
    void(*power_callback)(bool);
    void(*pos_report_callback)(std::vector<float>);
    void(*limit_switch_callback)(bool);
    void(*motion_complete_callback)(void);
    std::string result[MAX_CNT];
    //void(*callbacks[MAX_CNT])(std::string);
    std::function<void(std::string)> callbacks[MAX_CNT];
public:
    bool connected;
    int cnt;
    int cmd_pend_count;
    bool power_status;
    std::string port;
    uint32_t baudrate;
    uint8_t mode;
    std::string device_info[5];
        // device_info[0]: device_type
        // device_info[1]: hardware_version
        // device_info[2]: firmware_version
        // device_info[3]: api_version
        // device_info[4]: device_unique
public:
    /*!
    * Connect to uArm
    * @param port: a std::string containing the address of the serial port,
             which would be something like 'COM1' on Windows and '/dev/ttyS0' on Linux.
    * @param baudrate: An unsigned 32-bit integer that represents the baudrate
    * @param timeout: A Timeout struct that defines the timeout
    * return: 0 if connect success else -1
    */
    int connect(const std::string &port = "",
        uint32_t baudrate = 115200,
        Timeout timeout = Timeout::simpleTimeout(1000));

    /*!
    * Disconnect to uArm
    */
    void disconnect();

    /*!
    * Sync send cmd to uArm
    * @param cmd: a std::string
    * @param timeout: A float number that maximum seconds to waiting the execute result return
    * return: the execute result or TIMEOUT or NotConnected or Exception
    */
    std::string send_cmd_sync(const std::string &cmd, float timeout = default_timeout_2);

    /*!
    * Async send cmd to uArm
    * @param cmd: a std::string
    * @param callback: a function pointer to handle the execute result
    * return: OK or TIMEOUT or NotConnected or Exception
    */
    std::string send_cmd_async(const std::string &cmd, float timeout = default_timeout_2, std::function<void (std::string)> callback = NULL);
    std::string send_cmd_async(const std::string &cmd, float timeout = default_timeout_2, void(*callback)(std::string) = NULL);

    /*!
    * Register power event callback
    * @param callback: a function pointer
    * return: true/false
    */
    bool register_power_callback(void(*callback)(bool));

    /*!
    * Release power event callback
    */
    void release_power_callback();

    /*!
    * Register position report event callback
    * @param callback: a function pointer
    * return: true/false
    */
    bool register_report_position_callback(void(*callback)(std::vector<float>));

    /*!
    * Release position report event callback
    */
    void release_report_position_callback();

    /*!
    * Register limit switch event callback
    * @param callback: a function pointer
    * return: true/false
    */
    bool register_limit_switch_callback(void(*callback)(bool));

    /*!
    * Release limit switch event callback
    */
    void release_limit_switch_callback();

    /*!
    * Register motion-complete event callback
    * Called when an @9 V0 event is received from the firmware. Requires
    * set_motion_report(true) to be called first to enable the firmware
    * emitter.
    * @param callback: a function pointer
    * return: true/false
    */
    bool register_motion_complete_callback(void(*callback)(void));

    /*!
    * Release motion-complete event callback
    */
    void release_motion_complete_callback();

    /*
    * Report position in (interval) seconds
    * @param interval: seconds, disable report if interval is 0
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_report_position(float interval, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Enable or disable motion-complete event reports (@9 V0)
    * Sends M2122 V1/V0. When enabled, the firmware emits an @9 V0 event
    * when motion finishes (sys.state returns to STATE_IDLE after a move).
    * Pair with register_motion_complete_callback to consume the events.
    * @param on: true to enable, false to disable
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0 ok, negative on error (see set_position for codes)
    */
    int set_motion_report(bool on, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);
    
    /*
    * Attach the servo with the servo_id
    * @param servo_id: -1 ~ 3, -1 represents all servo
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_servo_attach(int servo_id, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Detach the servo with the servo_id
    * @param servo_id: -1 ~ 3, -1 represents all servo
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_servo_detach(int servo_id, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Set the mode, only support SwiftPro
    * @param mode: mode, 0: general mode, 1: laser mode, 2: 3D Print mode, 3: pen/gripper mode
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_mode(int mode, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Set the position
    * @param x: (mm) location X
    * @param y: (mm) location Y
    * @param x: (mm) location Z
    * @param speed: (mm/min) speed of move
    * @param relative: relative move or not, default is false
    * @param wait: true/false, default is false
    * @param timeout: timeout, default is 10s
    * @param callback: callback, default is None, only available if wait is true
    * @param cmd: "G0" or "G1", default is "G0"
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_position(float x, float y, float z, long speed = default_speed, bool relative = false,
        bool wait = false, float timeout = default_timeout_10, void(*callback)(int) = NULL, std::string cmd="G0");

    /*
    * Set the polar coordinate
    * @param stretch: (mm)
    * @param rotation: (degree), 0 ~ 180
    * @param height: (degree)
    * @param speed: (mm/min) speed of move
    * @param relative: relative move or not, default is false
    * @param wait: true/false, default is false
    * @param timeout: timeout, default is 10s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_polar(float stretch, float rotation, float height, long speed = default_speed, bool relative = false,
        bool wait = false, float timeout = default_timeout_10, void(*callback)(int) = NULL);

    /*
    * Set the servo angle
    * @param servo_id: servo_id, 0 ~ 3
    * @param angle: (degree), 0 ~ 180
    * @param speed: (mm/min) speed of move
    * @param relative: relative move or not, default is false
    * @param wait: true/false, default is false
    * @param timeout: timeout, default is 10s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_servo_angle(int servo_id, float angle, long speed = default_speed,
        bool wait = false, float timeout = default_timeout_10, void(*callback)(int) = NULL);

   /*
    * Move directly by joint angles (no inverse kinematics)
    * @param base: base motor angle (degrees), 0~180
    * @param left: left motor angle (degrees), 0~180
    * @param right: right motor angle (degrees), 0~180
    * @param speed: (mm/min) speed of move
    * @param wait: true/false, default is false
    * @param timeout: timeout, default is 10s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_joint_angles(float base, float left, float right, long speed = default_speed,
        bool wait = false, float timeout = default_timeout_10, void(*callback)(int) = NULL);

    /*
    * Pause arm motion (feed hold)
    * Sends S1000 V0. Motion can be resumed with resume_motion().
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0 ok, negative on error (see set_position for codes)
    */
    int pause_motion(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Resume arm motion after pause_motion()
    * Sends S1000 V1.
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0 ok, negative on error (see set_position for codes)
    */
    int resume_motion(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Hard reset of the motion control subsystem
    * Sends S1100. Equivalent to mc_reset() in the firmware. Aborts any
    * in-flight motion and clears the planner buffer. Distinct from reset(),
    * which is a homing move sequence.
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0 ok, negative on error (see set_position for codes)
    */
    int motion_reset(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Set the wrist angle (SERVO HAND)
    * @param angle: (degree), 0 ~ 180
    * @param speed: (mm/min) speed of move
    * @param relative: relative move or not, default is false
    * @param wait: true/false, default is false
    * @param timeout: timeout, default is 10s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_wrist(float angle, long speed = default_speed,
        bool wait = false, float timeout = default_timeout_10, void(*callback)(int) = NULL);

    /*
    * Control the buzzer
    * @param frequency: frequency, default is 1000
    * @param duration: duration, default is 2s
    * @param wait: true/false, default is false
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_buzzer(int frequency = 1000, float duration = 2,
        bool wait = false, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Control the pump
    * @param on: true/false
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_pump(bool on,
        bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Control the gripper
    * @param on: true/false
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_gripper(bool _catch,
        bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Set digital output value
    * @param pin: IO pin
    * @param value: 0: output low level, 1: output high level
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_digital_output(int pin, int value, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Set digital direction
    * @param pin: IO pin
    * @param value: 0: input, 1: output
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_digital_direction(int pin, int value, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Set the acceleration, only support firmware version > 4.0
    * @param acc: acc value
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return:
        0: ok
        -1: not connect
        -2: timeout
        -3: serial exception
        -4: command not exist
        -5: params error
        -6: address over
        -7: command buffer full
        -8: power is not connect
        -9: operatation failed
        -10: servo is detach, can not execute the cmd
    */
    int set_acceleration(float acc, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Reset
    * @param speed:
    * @param timeout: timeout, default is 10s
    * @param x: reset position x, default is 200
    * @param y: reset position x, default is 0
    * @param z: reset position x, default is 150
    */
    void reset(long speed = default_speed, float timeout = default_timeout_10, float x = 200, float y = 0, float z = 150);

    /*
    * Get position
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: std::vector<float> object, like [x, y, z] or []
    */
    std::vector<float> get_position(bool wait = true, float timeout = default_timeout_2, void(*callback)(std::vector<float>) = NULL);

    /*
    * Get polar
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: std::vector<float> object, like [stretch, rotation, height] or []
    */
    std::vector<float> get_polar(bool wait = true, float timeout = default_timeout_2, void(*callback)(std::vector<float>) = NULL);

    /*
    * Get servo angle
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: std::vector<float> object, like [base, left, right] or []
    */
    std::vector<float> get_servo_angle(bool wait = true, float timeout = default_timeout_2, void(*callback)(std::vector<float>) = NULL);

/*
    * Get the angle of a single joint
    * @param joint_id: 0=base, 1=left, 2=right, 3=end-effector (wrist)
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: angle in degrees, or -1 on failure
    */
    float get_joint_angle(int joint_id, bool wait = true, float timeout = default_timeout_2);

    /*
    * Convert Cartesian coordinates to joint angles (no movement)
    * @param x: (mm) location X
    * @param y: (mm) location Y
    * @param z: (mm) location Z
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: std::vector<float> object, like [base, left, right] or [] on failure
    */
    std::vector<float> coord_to_angles(float x, float y, float z,
        bool wait = true, float timeout = default_timeout_2, void(*callback)(std::vector<float>) = NULL);

    /*
    * Convert joint angles to Cartesian coordinates (no movement)
    * @param base: base motor angle (degrees)
    * @param left: left motor angle (degrees)
    * @param right: right motor angle (degrees)
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: std::vector<float> object, like [x, y, z] or [] on failure
    */
    std::vector<float> angles_to_coord(float base, float left, float right,
        bool wait = true, float timeout = default_timeout_2, void(*callback)(std::vector<float>) = NULL);

    /*
    * Check whether a target position is reachable
    * @param x: (mm) X, or (mm) stretch if polar=true
    * @param y: (mm) Y, or (degrees) rotation if polar=true
    * @param z: (mm) Z, or (mm) height if polar=true
    * @param polar: false=Cartesian, true=polar coordinates
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 1 reachable, 0 unreachable, negative on error
    */
    int is_reachable(float x, float y, float z, bool polar = false,
        bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);
        

    /*
    * Get device info
    * @param timeout: timeout, default is 10s
    * return: device_info
        // device_info[0]: device_type
        // device_info[1]: hardware_version
        // device_info[2]: firmware_version
        // device_info[3]: api_version
        // device_info[4]: device_unique
    */
    std::string* get_device_info(float timeout = default_timeout_10);

    /*
    * Get power status
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0: power off, 1: power on, -1: failed
    */
    int get_power_status(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Get mode
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: mode, 0: general mode, 1: laser mode, 2: 3D Print mode, 3: pen/gripper mode, -1: failed
    */
    int get_mode(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Get servo attach status
    * @param servo_id:
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0: attach, 1: detach, -1: failed
    */
    int get_servo_attach(int servo_id, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Get limit switch
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0: limit switch on, 1: limit switch off, -1: failed
    */
    int get_limit_switch(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Get gripper status
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0: stop, 1: working, 2: catch thing, -1: failed
    */
    int get_gripper_catch(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Get pump status
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0: stop, 1: working, 2: pump thing, -1: failed
    */
    int get_pump_status(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Get the digital value from specific pin
    * @param pin: specific pin
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0: low level, 1: high level, -1: failed
    */
    int get_digital(int pin, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Get the analog value from specific pin
    * @param pin: specific pin
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: analog value if return !== -1 else failed
    */
    int get_analog(int pin, bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Check uArm is moving or not
    * @param wait: true/false, default is true
    * @param timeout: timeout, default is 2s
    * @param callback: callback, default is None, only available if wait is true
    * return: 0: not move, 1: moving, -1: failed
    */
    int get_is_moving(bool wait = true, float timeout = default_timeout_2, void(*callback)(int) = NULL);

    /*
    * Wait until all command return or timeout
    */
    void flush_cmd();
private:
    inline std::string _send_cmd(const std::string &cmd, bool wait, float timeout, std::function<void(std::string)> callback = NULL);
    inline int _handle_set_int(std::string cmd, bool wait, float timeout, void(*callback)(int));
    inline std::vector<float> _handle_get_float_vector(std::string cmd, bool wait, float timeout, void(*callback)(std::vector<float>));
    inline int _handle_get_int(std::string cmd, bool wait, float timeout, void(*callback)(int), std::string type="");
    inline std::string _handle_get_string(std::string cmd, bool wait, float timeout, void(*callback)(std::string), std::string type = "");
};
} // namespace uarm

#endif // !_UARM_H_
