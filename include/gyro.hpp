#pragma once

#include "pros/imu.hpp"

#include <memory>

class Gyro {
    public:
        Gyro() = default;

        virtual void calibrate() = 0;

        virtual void setRotation(float rotation) = 0;

        virtual float getRotation() = 0;

        float getChange();
    private:
        float m_lastRotation = 0;
};

class V5Gyro : public Gyro {
    public:
        V5Gyro(std::shared_ptr<pros::IMU> imu);

        void calibrate() override;

        void setRotation(float rotation) override;

        float getRotation() override;
    private:
        std::shared_ptr<pros::IMU> m_imu;
};