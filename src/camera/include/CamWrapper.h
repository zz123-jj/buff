//
// Created by whoismz on 1/2/22.
//
#ifndef GMASTER_WM_NEW_CAMWRAPPER_H
#define GMASTER_WM_NEW_CAMWRAPPER_H
#include <cstdint>
#include <functional>
#include <vector>

class Camera {
   public:
    using FrameCallback = std::function<void(std::vector<uint8_t>&&, uint64_t hw_timestamp)>;

    virtual bool init(int roi_x, int roi_y, int roi_w, int roi_h, float exposure, float gain,
                      float FPS, int nBinning, int rotate_type = -1) = 0;

    virtual void setParam(float exposure, float gain) = 0;

    virtual bool start() = 0;

    virtual void stop() = 0;

    virtual bool init_is_successful() = 0;

    virtual void set_frame_callback(FrameCallback cb) = 0;

    /// Hardware timestamp tick frequency (Hz). Returns 0 if not available.
    virtual uint64_t tick_frequency() const { return 0; }

    virtual ~Camera() = default;
};

#endif  // GMASTER_WM_NEW_CAMWRAPPER_H
