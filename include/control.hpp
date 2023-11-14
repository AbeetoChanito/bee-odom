#pragma once

#include <memory>
#include <optional>

class SettleUtil {
    public:
        SettleUtil() = default;

        virtual bool isSettled(float error) = 0;
};

class Control {
    public:
        Control(std::unique_ptr<SettleUtil> settleUtil);

        std::optional<float> update(float error);
    protected:
        virtual float getUpdate(float error) = 0;

        std::unique_ptr<SettleUtil> m_settleUtil;
};

class TimeSettler : public SettleUtil {
    public:
        TimeSettler(float smallError, uint32_t smallErrorTime, float largeError, uint32_t largeErrorTime,
                    uint32_t maxTime);

        bool isSettled(float error) override;
    private:
        const float m_smallError;
        const uint32_t m_smallErrorTime;
        const float m_largeError;
        const uint32_t m_largeErrorTime;
        uint32_t m_maxTime;

        uint32_t m_smallTimer = 0;
        uint32_t m_largeTimer = 0;
        uint32_t m_maxTimer = 0;
};

class PID : public Control {
    public:
        PID(float kp, float ki, float kd, float iMax, std::unique_ptr<SettleUtil> settleUtil);
    private:
        float getUpdate(float error) override;

        const float m_kp;
        const float m_ki;
        const float m_kd;
        const float m_iMax;

        float m_prevError;
        float m_integral;
};