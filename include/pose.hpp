#pragma once

class Pose {
    public:
        float x;
        float y;
        float theta;

        Pose(float x, float y, float theta = 0);

        Pose operator+(const Pose& rhs);

        Pose operator-(const Pose& rhs);

        Pose operator*(float rhs);

        Pose operator/(float rhs);

        Pose rotateBy(float rhs);

        float distance(const Pose& rhs);

        float angleTo(const Pose& rhs);
};