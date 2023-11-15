#include "control.hpp"

#include <cmath>

#include "pros/rtos.hpp"

Control::Control(std::unique_ptr<SettleUtil> settleUtil)
    : m_settleUtil(std::move(settleUtil)) {}

std::optional<float> Control::update(float error) {
    if (m_settleUtil->isSettled(error)) { return std::nullopt; }

    return std::optional<float>(getUpdate(error));
}

TimeSettler::TimeSettler(float smallError, uint32_t smallErrorTime, float largeError, uint32_t largeErrorTime,
                         uint32_t maxTime)
    : m_smallError(smallError),
      m_smallErrorTime(smallErrorTime),
      m_largeError(largeError),
      m_largeErrorTime(largeErrorTime),
      m_maxTime(maxTime) {}

bool TimeSettler::isSettled(float error) {
    if (m_maxTimer == 0) { m_maxTimer = pros::millis(); }

    if (pros::millis() - m_maxTimer > m_maxTime) { return true; }

    if (std::fabs(error) < m_smallError) {
        if (m_smallTimer == 0) { m_smallTimer = pros::millis(); }

        if (pros::millis() - m_smallTimer > m_smallErrorTime) { return true; }
    } else {
        m_smallTimer = 0;
    }

    if (std::fabs(error) < m_largeError) {
        if (m_largeError == 0) { m_largeTimer = pros::millis(); }

        if (pros::millis() - m_largeTimer > m_largeErrorTime) { return true; }
    } else {
        m_largeTimer = 0;
    }

    return false;
}

PID::PID(float kp, float ki, float kd, float iMax, std::unique_ptr<SettleUtil> settleUtil)
    : Control(std::move(settleUtil)),
      m_kp(kp),
      m_ki(ki),
      m_kd(kd),
      m_iMax(iMax) {}

float PID::getUpdate(float error) {
    m_integral += error;

    if (std::fabs(error) > m_iMax || error == 0) { m_integral = 0; }

    const float derivative = error - m_prevError;
    m_prevError = error;

    return error * m_kp + m_integral * m_ki + derivative * m_kd;
}