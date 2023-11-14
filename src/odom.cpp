#include "odom.hpp"

#include <cmath>

float degToRad(float deg) { return deg / 180 * M_PI; }

float radToDeg(float rad) { return rad / M_PI * 180; }

Pose Odom::getPose() const { return m_pose; }

void Odom::setPose(const Pose& pose) { m_pose = pose; }

TwoEncoderImuOdom::TwoEncoderImuOdom(std::shared_ptr<TrackingWheel> leftTracker,
                                     std::shared_ptr<TrackingWheel> horzTracker, std::shared_ptr<Gyro> gyro)
    : m_leftTracker(leftTracker), m_horzTracker(horzTracker), m_gyro(gyro) {}

void TwoEncoderImuOdom::calibrate() {
    m_leftTracker->tare();
    m_horzTracker->tare();
    m_gyro->calibrate();
}

void TwoEncoderImuOdom::update() {
    float deltaHeading = degToRad(m_gyro->getChange());
    float avgHeading = m_pose.theta + deltaHeading / 2;

    float rawDx = m_horzTracker->getChange();
    float rawDy = m_leftTracker->getChange();

    float localDx, localDy;

    if (deltaHeading == 0) {
        localDx = rawDx;
        localDy = rawDy;
    } else {
        const float arcToLine = 2 * std::sin(deltaHeading / 2);
        localDx = arcToLine * (rawDx / deltaHeading + m_horzTracker->getOffset());
        localDy = arcToLine * (rawDy / deltaHeading + m_leftTracker->getOffset());
    }

    m_pose = m_pose + Pose(localDx, localDy).rotateBy(avgHeading);
    m_pose.theta += deltaHeading;
}