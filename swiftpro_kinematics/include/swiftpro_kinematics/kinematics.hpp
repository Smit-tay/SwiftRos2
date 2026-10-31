// kinematics.hpp
// Copyright 2026 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Analytical FK/IK for the UArm Swift Pro.
// Header-only — include directly, no linking required.
//
// Matches the SwiftPro-plusplus firmware (src/kin.cpp, src/kin.h), so results
// agree with M2220/M2221/M2222. The firmware remains the authority: it checks
// every move itself.
//
// ─────────────────────────────────────────────────────────────────────────────
// MODEL SUMMARY
// ─────────────────────────────────────────────────────────────────────────────
//
//   elbow:  r = SHOULDER_R + L1·cos(left)      z = SH_Z + L1·sin(left)
//   wrist:  r = elbow_r    + L2·cos(right)     z = elbow_z - L2·sin(right)
//   tip:    r = wrist_r    + DX                z = wrist_z - DZ
//
// DX, DZ are the firmware's defaults for mode 0 (no effector-specific offsets).
//
// Angle conventions (degrees), the "user" frame of P2200 and G2206:
//   base : 90° = straight ahead (+X). azimuth = base - 90°.
//          /joint_states and @3 use base - 90° instead: 0 = straight ahead.
//   left : lower arm above horizontal.
//   right: upper arm below horizontal.
//
// ─────────────────────────────────────────────────────────────────────────────
// USAGE
// ─────────────────────────────────────────────────────────────────────────────
//
//   #include "swiftpro_kinematics/kinematics.hpp"
//   using namespace swiftpro::kinematics;
//
//   // FK
//   double x, y, z;
//   fk(base_deg, left_deg, right_deg, x, y, z);
//
//   // IK
//   double base, left, right;
//   std::string msg;
//   if (!ik(x, y, z, base, left, right, msg)) { /* handle error */ }

#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace swiftpro::kinematics {

// ─────────────────────────────────────────────────────────────────────────────
// Firmware constants
// ─────────────────────────────────────────────────────────────────────────────

inline constexpr double L1         = 142.07;  // mm  lower arm
inline constexpr double L2         = 158.81;  // mm  upper arm
inline constexpr double SH_Z       = 107.40;  // mm  shoulder above the arm-frame origin
inline constexpr double SHOULDER_R =  13.20;  // mm  shoulder from the base axis
inline constexpr double DX         =  56.65;  // mm  wrist to tip, horizontal
inline constexpr double DZ         =  74.55;  // mm  wrist to tip, vertical down

// ─────────────────────────────────────────────────────────────────────────────
// Joint limits (degrees, user frame)
// ─────────────────────────────────────────────────────────────────────────────

inline constexpr double BASE_MIN = -5.0;
inline constexpr double BASE_MAX = 185.0;
inline constexpr double LEFT_MIN = -13.8;
inline constexpr double LEFT_MAX = 135.6;

// right's range depends on left: straight lines between these points, in tenths
// of a degree, at left = TABLE_LEFT_0 + i * TABLE_STEP.
inline constexpr double TABLE_LEFT_0 = -15.0;
inline constexpr double TABLE_STEP   = 5.0;
inline constexpr int    TABLE_POINTS = 32;
inline constexpr int    RIGHT_TABLE[TABLE_POINTS][2] = {
    {560, 1120},  {510, 1120},  {460, 1120},  {410, 1120},  {360, 1120},  {310, 1120},  {260, 1120},  // -15..15
    {210, 1120},  {160, 1110},  {110, 1100},  {60, 1090},   {10, 1080},   {-40, 1060},  {-80, 1040},  // 20..50
    {-120, 1020}, {-160, 1000}, {-180, 980},  {-180, 940},  {-180, 900},  {-180, 860},  {-180, 820},  // 55..85
    {-180, 780},  {-180, 730},  {-180, 680},  {-180, 630},  {-180, 580},  {-180, 530},  {-180, 480},  // 90..120
    {-180, 430},  {-180, 380},  {-180, 330},  {-180, 280},                                            // 125..140
};

inline void right_limits(double left_deg, double& lo, double& hi)
{
    const double pos = std::clamp((left_deg - TABLE_LEFT_0) / TABLE_STEP,
                                  0.0, static_cast<double>(TABLE_POINTS - 1));
    const int    i   = std::min(static_cast<int>(pos), TABLE_POINTS - 2);
    const double t   = pos - i;
    lo = 0.1 * (RIGHT_TABLE[i][0] + (RIGHT_TABLE[i + 1][0] - RIGHT_TABLE[i][0]) * t);
    hi = 0.1 * (RIGHT_TABLE[i][1] + (RIGHT_TABLE[i + 1][1] - RIGHT_TABLE[i][1]) * t);
}

// ─────────────────────────────────────────────────────────────────────────────
// Forward kinematics
// ─────────────────────────────────────────────────────────────────────────────

inline void fk(double base_deg, double left_deg, double right_deg,
               double& x,       double& y,       double& z)
{
    constexpr double DEG = M_PI / 180.0;

    const double r  = SHOULDER_R + L1 * std::cos(left_deg * DEG)
                    + L2 * std::cos(right_deg * DEG) + DX;
    const double az = (base_deg - 90.0) * DEG;

    x = r * std::cos(az);
    y = r * std::sin(az);
    z = SH_Z + L1 * std::sin(left_deg * DEG) - L2 * std::sin(right_deg * DEG) - DZ;
}

// ─────────────────────────────────────────────────────────────────────────────
// Inverse kinematics
// ─────────────────────────────────────────────────────────────────────────────
// Elbow-up solution, as in the firmware. Returns false if the target is outside
// the joint limits. msg is a human-readable description on failure, "OK" on
// success.

inline bool ik(double x,         double y,         double z,
               double& base_deg, double& left_deg, double& right_deg,
               std::string& msg)
{
    constexpr double RAD = 180.0 / M_PI;

    const double heading = std::atan2(y, x);
    base_deg = 90.0 + heading * RAD;

    if (base_deg < BASE_MIN || base_deg > BASE_MAX) {
        msg = "Base out of range: " + std::to_string(base_deg) + "deg";
        return false;
    }

    // Planar problem in the arm's vertical plane, relative to the shoulder.
    const double s  = std::hypot(x, y) - DX - SHOULDER_R;
    const double h  = z + DZ - SH_Z;
    const double d2 = s*s + h*h;
    const double d  = std::sqrt(d2);

    const double ratio = (d > 0.0) ? (L1*L1 + d2 - L2*L2) / (2.0 * L1 * d) : 2.0;
    if (std::abs(ratio) > 1.0) {
        msg = "Unreachable — no IK solution exists";
        return false;
    }

    const double left = std::atan2(h, s) + std::acos(ratio);
    left_deg  = left * RAD;
    right_deg = std::atan2(-(h - L1 * std::sin(left)), s - L1 * std::cos(left)) * RAD;

    if (left_deg < LEFT_MIN || left_deg > LEFT_MAX) {
        msg = "Left out of range: " + std::to_string(left_deg) + "deg";
        return false;
    }

    double lo = 0.0, hi = 0.0;
    right_limits(left_deg, lo, hi);
    if (right_deg < lo || right_deg > hi) {
        msg = "Right out of range: " + std::to_string(right_deg)
            + "deg at left=" + std::to_string(left_deg)
            + "deg (" + std::to_string(lo) + " to " + std::to_string(hi) + ")";
        return false;
    }

    msg = "OK";
    return true;
}

}  // namespace swiftpro::kinematics
