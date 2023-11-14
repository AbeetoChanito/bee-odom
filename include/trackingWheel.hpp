#pragma once

#include "pros/motors.hpp"
#include "pros/rotation.hpp"

#include <memory>

class TrackingWheel {
    public:
        TrackingWheel(float offset);

        void tare();

        virtual float getPosition() const = 0;

        float getOffset() const;

        float getChange();
    protected:
        virtual void tareDevice() = 0;
    private:
        float m_offset;
        float m_lastPosition;
};

class MotorTracker : public TrackingWheel {
    public:
        MotorTracker(std::shared_ptr<pros::MotorGroup> motor, float wheelDiameter, float rpm, float offset);

        float getPosition() const override;
    private:
        void tareDevice() override;

        std::shared_ptr<pros::MotorGroup> m_motor;
        float m_wheelDiameter;
        float m_rpm;
};

class RotationTracker : public TrackingWheel {
    public:
        RotationTracker(std::shared_ptr<pros::Rotation> rotation, float wheelDiameter, float gearRatio, float offset);

        float getPosition() const override;
    private:
        void tareDevice() override;

        std::shared_ptr<pros::Rotation> m_rotation;
        float m_wheelDiameter;
        float m_gearRatio;
};