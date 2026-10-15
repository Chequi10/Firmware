#pragma once
#include <stdint.h>
#include <limits.h>

// Input is a coherent snapshot taken with interrupts briefly masked.
struct SpeedPulseSnapshot
{
    uint64_t total_pulses = 0;
    uint32_t last_pulse_ms = 0;
    uint32_t period_ms = 0;
    uint32_t uptime_ms = 0;
    bool seen_pulse = false;
    bool period_valid = false;
};

class SpeedPulseEstimator
{
public:
    static constexpr uint32_t STOP_TIMEOUT_MS = 1000;

    uint32_t update(const SpeedPulseSnapshot& s)
    {
        if (!initialized || s.total_pulses < previous_pulses)
        {
            initialized = true;
            previous_pulses = s.total_pulses;
            previous_ms = s.uptime_ms;
            return 0;
        }

        const uint32_t elapsed = s.uptime_ms - previous_ms;
        const uint64_t delta = s.total_pulses - previous_pulses;
        previous_pulses = s.total_pulses;
        previous_ms = s.uptime_ms;

        if (!s.seen_pulse || s.uptime_ms - s.last_pulse_ms >= STOP_TIMEOUT_MS)
            return 0;

        // Frequent pulses: average over the telemetry interval. Multiple
        // edges in the same millisecond remain counted.
        uint64_t frequency = 0;
        if (delta >= 2 && elapsed != 0)
        {
            // delta is bounded by physical edge rate and a 100 ms interval.
            if (delta > UINT64_MAX / 1000000ULL) return UINT32_MAX;
            frequency = (delta * 1000000ULL) / elapsed;
        }
        else if (s.period_valid && s.period_ms != 0)
        {
            // Sparse pulses: period measurement avoids alternating zero
            // with spikes. As the next pulse becomes late, speed decays.
            uint32_t period = s.period_ms;
            const uint32_t age = s.uptime_ms - s.last_pulse_ms;
            if (age > period) period = age;
            frequency = 1000000ULL / period;
        }
        return frequency > UINT32_MAX ? UINT32_MAX : uint32_t(frequency);
    }

private:
    bool initialized = false;
    uint64_t previous_pulses = 0;
    uint32_t previous_ms = 0;
};

// Payload P used by the Qt GUI: 18 bytes, all multibyte fields big-endian.
inline void SpeedPulseEncode(uint8_t *p, const SpeedPulseSnapshot& s,
                             uint32_t frequency_millihz)
{
    p[0] = 'P';
    p[1] = 1; // Acquisition initialized; does not diagnose sensor wiring.
    for (unsigned i = 0; i < 8; ++i)
        p[2 + i] = uint8_t(s.total_pulses >> (56 - 8 * i));
    for (unsigned i = 0; i < 4; ++i)
    {
        p[10 + i] = uint8_t(frequency_millihz >> (24 - 8 * i));
        p[14 + i] = uint8_t(s.uptime_ms >> (24 - 8 * i));
    }
}
