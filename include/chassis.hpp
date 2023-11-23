#pragma once

#include "pros/motors.hpp"

#include "odom.hpp"
#include "control.hpp"
#include "pose.hpp"

class Chassis {
    public:
        Chassis(
            std::shared_ptr<pros::MotorGroup> leftMotors, 
            std::shared_ptr<pros::MotorGroup> rightMotors,
            std::shared_ptr<Odom> odom,
            std::unique_ptr<PD> lateralPD,
            std::unique_ptr<PD> angularPD
        );

        void turnToHeading(float target);
        void turnToPoint(const Pose& target);
        void moveToPoint(const Pose& target);

        void tank(float left, float right);
        void arcade(float forward, float turn);
    private:
        std::shared_ptr<pros::MotorGroup> m_leftMotors;
        std::shared_ptr<pros::MotorGroup> m_rightMotors;
        std::shared_ptr<Odom> m_odom;
        std::unique_ptr<PD> m_lateralPD;
        std::unique_ptr<PD> m_angularPD;
};