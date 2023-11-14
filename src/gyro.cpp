#include "gyro.hpp"

float Gyro::getChange() {
    const float rotation = getRotation();
    const float change = rotation - m_lastRotation;
    m_lastRotation = rotation;
    return change;
}

V5Gyro::V5Gyro(std::shared_ptr<pros::IMU> imu) : m_imu(imu) {}

void V5Gyro::calibrate() { m_imu->reset(true); }

void V5Gyro::setRotation(float rotation) { m_imu->set_rotation(rotation); }

float V5Gyro::getRotation() { return m_imu->get_rotation(); }