#pragma once
#include <array>

namespace kinematics {

// ============================================================================
// Physical link lengths (meters), shaft-center to shaft-center, measured
// directly on the physical master arm.
// ============================================================================
constexpr double L1 = 0.043;  // J1 -> J2
constexpr double L2 = 0.037;  // J2 -> J3
constexpr double L3 = 0.043;  // J3 -> J4   (roughly the "shoulder-to-elbow" span)
constexpr double L4 = 0.037;  // J4 -> J5
constexpr double L5 = 0.043;  // J5 -> J6   (roughly the "elbow-to-wrist" span)
constexpr double L6 = 0.036;  // J6 -> J7
constexpr double L7 = 0.033;  // J7 -> Hand

// Joint pattern (confirmed against the Kinova Gen3's own convention):
//   J1 roll, J2 bend, J3 roll, J4 bend, J5 roll, J6 bend, J7 roll.

// Master hand pose: position (meters) + orientation (radians, ZYX Euler).
struct Pose {
    double x, y, z;
    double roll, pitch, yaw;
};

// 4x4 homogeneous transform, row-major.
struct Mat4 {
    double m[4][4];
};

Mat4 identity();
Mat4 multiply(const Mat4& a, const Mat4& b);
Mat4 rotZ(double theta);    // roll joints: rotate about local Z (current forward axis)
Mat4 rotX(double theta);    // bend joints: rotate about local X (perpendicular)
Mat4 translateZ(double d);  // link translation along local Z (forward)

// Forward kinematics: given the 7 joint angles (radians, J1..J7), compute
// the master hand's pose relative to the base (J1) mount frame.
Pose forward_kinematics(const std::array<double, 7>& joint_angles);

} // namespace kinematics
