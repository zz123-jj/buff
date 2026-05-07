#pragma once

#include <chrono>
#include <cmath>

class LowPassFilter {
   public:
    explicit LowPassFilter(float cutoff_hz = 1.0f) : cutoff_(cutoff_hz) {
        y_ = 0.0f;
        last_ts_ = std::chrono::steady_clock::now();
    }

    /**
     * Update filter with a new sample, dt given explicitly (recommended for real‑time loops).
     * @param x   – raw input sample
     * @param dt  – sample period in seconds
     * @return filtered value
     */
    float update(float x, float dt) {
        float a = alpha(dt);
        y_ += a * (x - y_);
        return y_;
    }

    /**
     * Update filter with a new sample, dt measured internally via std::chrono.
     * Suitable for non‑real‑time PC code; avoid on bare‑metal MCUs.
     */
    float update(float x) {
        using clock = std::chrono::steady_clock;
        auto now = clock::now();
        float dt = std::chrono::duration<float>(now - last_ts_).count();
        last_ts_ = now;
        return update(x, dt);
    }

    /** Reset the filter to a specific value. */
    void reset(float value) { y_ = value; }

    /** Change cut‑off frequency at runtime. */
    void setCutoff(float cutoff_hz) { cutoff_ = cutoff_hz; }

    /** change the initial angel */
    void set_initial(const float& new_value) { this->y_ = new_value; }

    /** Get current filtered output (without updating). */
    float value() const { return y_; }

    /** get the initial angle
     * used in the first filter start
     */
    float get_initial() { return this->y_; }

    void configure(float cutoff_hz) { cutoff_ = cutoff_hz; }

   private:
    /** Compute smoothing factor α from dt. */
    float alpha(float dt) const {
        // RC = 1 / (2πf_c); α = dt / (RC + dt)
        float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * cutoff_);
        return dt / (rc + dt);
    }

    float cutoff_;  ///< cut‑off frequency in Hz
    float y_;       ///< current filtered value
    std::chrono::steady_clock::time_point last_ts_;
};
