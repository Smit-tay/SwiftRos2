// kinematics.hpp
// Copyright 2025 Jack Sidman Smith
// Licensed under the MIT License. See LICENSE in project root.
//
// Analytical FK/IK for the UArm Swift Pro.
// Header-only — include directly, no linking required.
//
// ─────────────────────────────────────────────────────────────────────────────
// MODEL SUMMARY
// ─────────────────────────────────────────────────────────────────────────────
//
// The parallel linkage decouples the forearm world angle from J2.
// Forearm world angle depends ONLY on J3.
//
//   shoulder:  r = SHOULDER_R,  z = SH_Z
//   elbow:     r = SHOULDER_R + L1·cos(J2°)
//              z = SH_Z       + L1·sin(J2°)
//   J8 pivot:  r = elbow_r + L2·cos(-J3°)
//              z = elbow_z + L2·sin(-J3°)
//   tip:       r = J8_r + DX
//              z = J8_z - DZ
//
// Firmware angle conventions (degrees, zero offsets):
//   J1: 90° = arm pointing along +X. azimuth = J1 - 90°.
//   J2: upper arm angle above horizontal. 0° = horizontal, 90° = vertical.
//   J3: forearm downward angle. 0° = horizontal, +° = tip moves down.
//
// Calibrated from 16-point commanded sweep, RMS = 0.27mm.
//
// ─────────────────────────────────────────────────────────────────────────────
// USAGE
// ─────────────────────────────────────────────────────────────────────────────
//
//   #include "swiftpro_resources/kinematics.hpp"
//   using namespace swiftpro::kinematics;
//
//   // FK
//   double x, y, z;
//   fk(j1_deg, j2_deg, j3_deg, x, y, z);
//
//   // IK
//   double j1, j2, j3;
//   std::string msg;
//   if (!ik(x, y, z, j1, j2, j3, msg)) { /* handle error */ }

#pragma once

#include <cmath>
#include <string>

namespace swiftpro::kinematics {

// ─────────────────────────────────────────────────────────────────────────────
// Calibrated constants
// ─────────────────────────────────────────────────────────────────────────────

inline constexpr double L1         = 143.00;  // mm  upper arm (shoulder→elbow)
inline constexpr double L2         = 158.45;  // mm  forearm (elbow→J8 pivot)
inline constexpr double SH_Z       = 115.50;  // mm  shoulder pivot above table
inline constexpr double SHOULDER_R =  13.20;  // mm  shoulder pivot fwd of J1
inline constexpr double DX         =  56.78;  // mm  J8→tip, horizontal
inline constexpr double DZ         =  84.45;  // mm  J8→tip, vertical down

// ─────────────────────────────────────────────────────────────────────────────
// Workspace limits (firmware degrees / mm)
// ─────────────────────────────────────────────────────────────────────────────

inline constexpr double J1_MIN    =  15.0;
inline constexpr double J1_MAX    = 165.0;
inline constexpr double J2_MIN    =  20.0;
inline constexpr double J2_MAX    = 120.0;
inline constexpr double J3_MIN    =   0.0;
inline constexpr double J3_MAX    =  70.0;
inline constexpr double REACH_MIN = 133.0;   // mm from J1 axis
inline constexpr double REACH_MAX = 340.0;   // mm from J1 axis
inline constexpr double Z_MIN     =  5.0;   // mm safety floor
inline constexpr double Z_MAX     = 250.0;   // mm

// ─────────────────────────────────────────────────────────────────────────────
// Linkage self-collision constraints (empirically calibrated)
// ─────────────────────────────────────────────────────────────────────────────
// Two independent constraints must both be satisfied:
//   1. J3 ≥ (90 - J2) * 0.43  — upper arm vs parallel linkage connector clash
//   2. J2 + J3 ≥ 52°          — elbow over-centre (linkage past 180°)
// Use j3_safe_min() to query the effective minimum J3 for a given J2.

inline constexpr double CLASH_SLOPE    = 0.43;  // J3_min = (90 - J2) * slope
inline constexpr double ELBOW_SUM_MIN  = 52.0;  // J2 + J3 must exceed this

inline double j3_safe_min(double j2_deg)
{
    const double c1 = (90.0 - j2_deg) * CLASH_SLOPE;
    const double c2 = ELBOW_SUM_MIN - j2_deg;
    return std::max({0.0, c1, c2});
}

// Returns true if (J2, J3) is in the safe zone.
inline bool joints_safe(double j2_deg, double j3_deg)
{
    return j3_deg >= j3_safe_min(j2_deg);
}


// ─────────────────────────────────────────────────────────────────────────────
// Forward kinematics — 2D (vertical plane)
// ─────────────────────────────────────────────────────────────────────────────
// Returns tip_r (horizontal reach from J1 axis) and tip_z (height above table).

inline void fk_2d(double j2_deg, double j3_deg,
                  double& tip_r,  double& tip_z)
{
    const double a2 = j2_deg * (M_PI / 180.0);
    const double fw = -j3_deg * (M_PI / 180.0);

    const double el_r = SHOULDER_R + L1 * std::cos(a2);
    const double el_z = SH_Z       + L1 * std::sin(a2);

    const double j8_r = el_r + L2 * std::cos(fw);
    const double j8_z = el_z + L2 * std::sin(fw);

    tip_r = j8_r + DX;
    tip_z = j8_z - DZ;
}

// ─────────────────────────────────────────────────────────────────────────────
// Forward kinematics — 3D (firmware frame)
// ─────────────────────────────────────────────────────────────────────────────

inline void fk(double j1_deg, double j2_deg, double j3_deg,
               double& x,     double& y,     double& z)
{
    double tip_r, tip_z;
    fk_2d(j2_deg, j3_deg, tip_r, tip_z);

    const double az = (j1_deg - 90.0) * (M_PI / 180.0);
    x = tip_r * std::cos(az);
    y = tip_r * std::sin(az);
    z = tip_z;
}

// ─────────────────────────────────────────────────────────────────────────────
// Inverse kinematics — 3D (firmware frame)
// ─────────────────────────────────────────────────────────────────────────────
// Returns false if target is outside the reachable workspace.
// msg is set to a human-readable description on failure, "OK" on success.

inline bool ik(double x,       double y,       double z,
               double& j1_deg, double& j2_deg, double& j3_deg,
               std::string& msg)
{
    constexpr double RAD = 180.0 / M_PI;
    constexpr double DEG = M_PI / 180.0;

    // ── J1: azimuth ──────────────────────────────────────────────────────────
    j1_deg = 90.0 + std::atan2(y, x) * RAD;

    if (j1_deg < J1_MIN || j1_deg > J1_MAX) {
        msg = "J1 out of range: " + std::to_string(j1_deg) + "deg";
        return false;
    }

    // ── Reach and Z limits ───────────────────────────────────────────────────
    const double tip_r = std::sqrt(x*x + y*y);

    if (tip_r < REACH_MIN) {
        msg = "Too close: reach=" + std::to_string(tip_r)
            + "mm (min=" + std::to_string(REACH_MIN) + "mm)";
        return false;
    }
    if (tip_r > REACH_MAX) {
        msg = "Too far: reach=" + std::to_string(tip_r)
            + "mm (max=" + std::to_string(REACH_MAX) + "mm)";
        return false;
    }
    if (z < Z_MIN) {
        msg = "Below safety floor: z=" + std::to_string(z)
            + "mm (min=" + std::to_string(Z_MIN) + "mm)";
        return false;
    }
    if (z > Z_MAX) {
        msg = "Too high: z=" + std::to_string(z)
            + "mm (max=" + std::to_string(Z_MAX) + "mm)";
        return false;
    }

    // ── 2D IK ────────────────────────────────────────────────────────────────
    //
    // R = tip_r - DX - SHOULDER_R  = L1·cos(J2) + L2·cos(J3)
    // H = tip_z + DZ - SH_Z        = L1·sin(J2) - L2·sin(J3)
    //
    // Eliminate J2 by squaring and adding:
    //   L1² = (R - L2·cos(J3))² + (H + L2·sin(J3))²
    //
    // Rearrange to A·cos(J3) + B·sin(J3) = C:
    //   A =  2·R·L2
    //   B = -2·H·L2
    //   C =  R² + H² + L2² - L1²
    //
    // Solve: J3 = -(atan2(-B, A) ± acos(C / sqrt(A²+B²)))

    const double R = tip_r - DX - SHOULDER_R;
    const double H = z     + DZ - SH_Z;

    const double A = 2.0 * R * L2;
    const double B = -2.0 * H * L2;
    const double C = R*R + H*H + L2*L2 - L1*L1;
    const double M = std::sqrt(A*A + B*B);

    if (M < 1e-9) {
        msg = "Degenerate IK — target on J1 axis";
        return false;
    }

    const double ratio = C / M;
    if (std::abs(ratio) > 1.0) {
        msg = "Unreachable — no IK solution exists";
        return false;
    }

    const double base  = std::atan2(-B, A);
    const double delta = std::acos(ratio);

    const double j3_a = -(base + delta) * RAD;
    const double j3_b = -(base - delta) * RAD;

    const bool a_ok = (j3_a >= J3_MIN && j3_a <= J3_MAX);
    const bool b_ok = (j3_b >= J3_MIN && j3_b <= J3_MAX);

    if (!a_ok && !b_ok) {
        msg = "No solution within J3 limits ["
            + std::to_string(J3_MIN) + ", " + std::to_string(J3_MAX)
            + "]. Candidates: " + std::to_string(j3_a)
            + ", " + std::to_string(j3_b);
        return false;
    }

    // Prefer smaller J3 (less elbow bend = more stable pose)
    if (a_ok && b_ok)
        j3_deg = (std::abs(j3_a) <= std::abs(j3_b)) ? j3_a : j3_b;
    else
        j3_deg = a_ok ? j3_a : j3_b;

    // Recover J2 from known J3
    const double j3_rad = j3_deg * DEG;
    j2_deg = std::atan2(H + L2 * std::sin(j3_rad),
                        R - L2 * std::cos(j3_rad)) * RAD;

    if (j2_deg < J2_MIN || j2_deg > J2_MAX) {
        msg = "J2 out of range: " + std::to_string(j2_deg) + "deg";
        return false;
    }
    // ── Linkage self-collision check ─────────────────────────────────────────
    if (!joints_safe(j2_deg, j3_deg)) {
        msg = "Solution violates linkage constraints: J2=" + std::to_string(j2_deg)
            + " J3=" + std::to_string(j3_deg)
            + " (J3_min=" + std::to_string(j3_safe_min(j2_deg)) + ")";
        return false;
    }
    
    msg = "OK";
    return true;
}

}  // namespace swiftpro::kinematics
