#include "pose.hpp"

#include <cmath>

Pose::Pose(float x, float y, float theta)
    : x(x),
      y(y),
      theta(theta) {}

Pose Pose::operator+(const Pose& rhs) const { return Pose(x + rhs.x, y + rhs.y, theta); }

Pose Pose::operator-(const Pose& rhs) const { return Pose(x - rhs.x, y - rhs.y, theta); }

Pose Pose::operator*(float rhs) const { return Pose(x * rhs, y * rhs, theta); }

Pose Pose::operator/(float rhs) const { return Pose(x / rhs, y / rhs, theta); }

Pose Pose::rotateBy(float rhs) const {
    const float s = std::sin(rhs);
    const float c = std::cos(rhs);
    return Pose(x * c - y * s, x * s + y * c, theta);
}

float Pose::distance(const Pose& rhs) const { return std::hypot(x - rhs.x, y - rhs.y); }

float Pose::angleTo(const Pose& rhs) const {
    // note: bee-odom uses compass angles
    return std::atan2(x - rhs.x, y - rhs.y);
}