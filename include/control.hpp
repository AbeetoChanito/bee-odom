#pragma once

#include <optional>
#include <cstdint>

class PD {
    public:
        PD(
            float kp,
            float kd,
            float smallError, 
            uint32_t smallErrorTime, 
            float largeError, 
            uint32_t largeErrorTime,
            uint32_t maxTime
        );

        void reset();

        float getUpdate(float error);

        bool isSettled();
    private:
        const float m_kp;
        const float m_kd;

        float m_prevError;

        const float m_smallError;
        const uint32_t m_smallErrorTime;
        const float m_largeError;
        const uint32_t m_largeErrorTime;
        uint32_t m_maxTime;

        uint32_t m_smallTimer = 0;
        uint32_t m_largeTimer = 0;
        uint32_t m_maxTimer = 0;
};