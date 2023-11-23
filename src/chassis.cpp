#include "chassis.hpp"

#include <cmath>

static float rollAngle180(float angle) {
    return std::remainder(angle, 2 * M_PI);
}

static float rollAngle90(float angle) {
    return std::remainder(angle, M_PI);
}

Chassis::Chassis(
    std::shared_ptr<pros::MotorGroup> leftMotors, 
    std::shared_ptr<pros::MotorGroup> rightMotors,
    std::shared_ptr<Odom> odom,
    std::unique_ptr<PD> lateralPD,
    std::unique_ptr<PD> angularPD
) : m_leftMotors(leftMotors), m_rightMotors(rightMotors), m_odom(odom), m_lateralPD(std::move(lateralPD)), m_angularPD(std::move(angularPD)) {
    
}

void Chassis::tank(float left, float right) {
    m_leftMotors->move(static_cast<int>(left));
    m_rightMotors->move(static_cast<int>(right));
}

void Chassis::arcade(float forward, float turn) {
    float left = forward + turn;
    float right = forward - turn;

    float mag = std::max(std::abs(left) / 127.0, std::abs(right) / 127.0);

    if (mag > 1) {
        left /= mag;
        right /= mag;
    }

    tank(left, right);
}

void Chassis::turnToHeading(float target) {
    m_angularPD->reset();

    while (!m_angularPD->isSettled()) {
        float headingError = rollAngle180(target - m_odom->getPose().theta);
        float angularOutput = m_angularPD->getUpdate(headingError);

        arcade(0, angularOutput);

        pros::delay(10);
    }

    arcade(0, 0);
}

void Chassis::turnToPoint(const Pose& target) {
    m_angularPD->reset();

    while (!m_angularPD->isSettled()) {
        Pose pose = m_odom->getPose();
        float dx = target.x - pose.x;
        float dy = target.y - pose.y;

        float headingError = rollAngle180(std::atan2(dx, dy) - pose.theta);
        float angularOutput = m_angularPD->getUpdate(headingError);

        arcade(0, angularOutput);

        pros::delay(10);
    }

    arcade(0, 0);
}

void Chassis::moveToPoint(const Pose& target) {
    m_lateralPD->reset();
    m_angularPD->reset();
    
    while (!m_lateralPD->isSettled()) {
        Pose pose = m_odom->getPose();

        float dx = target.x - pose.x;
        float dy = target.y - pose.y;

        float driveError = std::hypot(dx, dy);
        float headingError = rollAngle180(std::atan2(dx, dy));

        float lateralOutput = m_lateralPD->getUpdate(driveError) * std::cos(headingError);
        float angularOutput = m_angularPD->getUpdate(rollAngle90(headingError));

        arcade(lateralOutput, angularOutput);

        pros::delay(10);
    }

    arcade(0, 0);
}