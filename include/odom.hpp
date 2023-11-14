#pragma once

#include "gyro.hpp"
#include "pose.hpp"
#include "trackingWheel.hpp"

class Odom {
    public:
        Odom() = default;

        virtual void calibrate() = 0;

        Pose getPose() const;

        void setPose(const Pose& pose);

        virtual void update() = 0;
    protected:
        Pose m_pose = Pose(0, 0);
};

class TwoEncoderImuOdom : public Odom {
    public:
        TwoEncoderImuOdom(std::shared_ptr<TrackingWheel> leftTracker, std::shared_ptr<TrackingWheel> horzTracker,
                          std::shared_ptr<Gyro> gyro);

        void calibrate() override;

        void update() override;
    private:
        std::shared_ptr<TrackingWheel> m_leftTracker;
        std::shared_ptr<TrackingWheel> m_horzTracker;
        std::shared_ptr<Gyro> m_gyro;
};

float degToRad(float deg);
float radToDeg(float rad);