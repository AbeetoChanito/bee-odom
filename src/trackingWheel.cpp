#include "trackingWheel.hpp"

#include <cmath>

TrackingWheel::TrackingWheel(float offset) : m_offset(offset) {}

float TrackingWheel::getOffset() const { return m_offset; }

float TrackingWheel::getChange() {
    const float position = getPosition();
    const float change = position - m_lastPosition;
    m_lastPosition = position;
    return change;
}

void TrackingWheel::tare() {
    tareDevice();
    m_lastPosition = 0;
}

MotorTracker::MotorTracker(std::shared_ptr<pros::MotorGroup> motor, float wheelDiameter, float rpm, float offset)
    : TrackingWheel(offset), m_motor(motor), m_wheelDiameter(wheelDiameter), m_rpm(rpm) {}

void MotorTracker::tareDevice() { m_motor->tare_position(); }

float MotorTracker::getPosition() const {
    float rpm;

    switch (m_motor->get_gearing()[0]) {
        case pros::E_MOTOR_GEAR_100: {
            rpm = 100;
            break;
        }
        case pros::E_MOTOR_GEAR_200: {
            rpm = 200;
            break;
        }
        case pros::E_MOTOR_GEAR_600: {
            rpm = 600;
            break;
        }
        default: {
            __builtin_unreachable();
            break;
        }
    }

    m_motor->set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
    return m_motor->get_positions()[0] * (m_rpm / rpm) * m_wheelDiameter * M_PI;
}

RotationTracker::RotationTracker(std::shared_ptr<pros::Rotation> rotation, float wheelDiameter, float gearRatio,
                                 float offset)
    : TrackingWheel(offset), m_rotation(rotation), m_wheelDiameter(wheelDiameter), m_gearRatio(gearRatio) {}

void RotationTracker::tareDevice() { m_rotation->reset_position(); }

float RotationTracker::getPosition() const {
    return m_rotation->get_position() / 36000.0f * m_gearRatio * m_wheelDiameter * M_PI;
}