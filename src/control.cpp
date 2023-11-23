#include "control.hpp"

#include <cmath>

#include "pros/rtos.hpp"

PD::PD(float kp, float kd, float smallError, uint32_t smallErrorTime, float largeError, uint32_t largeErrorTime,
         uint32_t maxTime)
    : m_kp(kp),
    m_kd(kd),
    m_smallError(smallError),
    m_smallErrorTime(smallErrorTime),
    m_largeError(largeError),
    m_largeErrorTime(largeErrorTime),
    m_maxTime(maxTime) {}

void PD::reset() {
    m_maxTimer = 0;
    m_smallTimer = 0;
    m_largeTimer = 0;
    m_prevError = 0;
}

bool PD::isSettled() {
    if (m_maxTimer == 0) { m_maxTimer = pros::millis(); }

    if (pros::millis() - m_maxTimer > m_maxTime) { return true; }

    if (std::fabs(m_prevError) < m_smallError) {
        if (m_smallTimer == 0) { m_smallTimer = pros::millis(); }

        if (pros::millis() - m_smallTimer > m_smallErrorTime) { return true; }
    } else {
        m_smallTimer = 0;
    }

    if (std::fabs(m_prevError) < m_largeError) {
        if (m_largeError == 0) { m_largeTimer = pros::millis(); }

        if (pros::millis() - m_largeTimer > m_largeErrorTime) { return true; }
    } else {
        m_largeTimer = 0;
    }

    return false;
}

float PD::getUpdate(float error) {
    const float derivative = error - m_prevError;
    m_prevError = error;

    return error * m_kp + derivative * m_kd;
}