#include "kinematics.h"
#include <cmath>

namespace kinematics {

Mat4 identity() {
    Mat4 r{};
    for (int i = 0; i < 4; i++) r.m[i][i] = 1.0;
    return r;
}

Mat4 multiply(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            double sum = 0.0;
            for (int k = 0; k < 4; k++) sum += a.m[i][k] * b.m[k][j];
            r.m[i][j] = sum;
        }
    return r;
}

Mat4 rotZ(double theta) {
    Mat4 r = identity();
    double c = std::cos(theta), s = std::sin(theta);
    r.m[0][0] = c; r.m[0][1] = -s;
    r.m[1][0] = s; r.m[1][1] = c;
    return r;
}

Mat4 rotX(double theta) {
    Mat4 r = identity();
    double c = std::cos(theta), s = std::sin(theta);
    r.m[1][1] = c; r.m[1][2] = -s;
    r.m[2][1] = s; r.m[2][2] = c;
    return r;
}

Mat4 translateZ(double d) {
    Mat4 r = identity();
    r.m[2][3] = d;
    return r;
}

Pose forward_kinematics(const std::array<double, 7>& q) {
    // Chain each joint: rotate about its own axis, then translate to the
    // next joint along the CURRENT forward direction (local Z). Roll
    // joints rotate about Z (don't change forward direction, but twist
    // everything downstream); bend joints rotate about X (do change
    // the forward direction for everything downstream).
    Mat4 T = identity();
    T = multiply(T, rotZ(q[0]));       // J1 roll
    T = multiply(T, translateZ(L1));
    T = multiply(T, rotX(q[1]));       // J2 bend
    T = multiply(T, translateZ(L2));
    T = multiply(T, rotZ(q[2]));       // J3 roll
    T = multiply(T, translateZ(L3));
    T = multiply(T, rotX(q[3]));       // J4 bend
    T = multiply(T, translateZ(L4));
    T = multiply(T, rotZ(q[4]));       // J5 roll
    T = multiply(T, translateZ(L5));
    T = multiply(T, rotX(q[5]));       // J6 bend
    T = multiply(T, translateZ(L6));
    T = multiply(T, rotZ(q[6]));       // J7 roll
    T = multiply(T, translateZ(L7));   // to the hand point

    Pose p{};
    p.x = T.m[0][3];
    p.y = T.m[1][3];
    p.z = T.m[2][3];

    // Extract roll/pitch/yaw (ZYX Euler) from the rotation part of T.
    p.pitch = std::asin(-T.m[2][0]);
    if (std::abs(std::cos(p.pitch)) > 1e-6) {
        p.roll = std::atan2(T.m[2][1], T.m[2][2]);
        p.yaw  = std::atan2(T.m[1][0], T.m[0][0]);
    } else {
        p.roll = std::atan2(-T.m[1][2], T.m[1][1]);  // gimbal-lock fallback
        p.yaw  = 0.0;
    }
    return p;
}

} // namespace kinematics
