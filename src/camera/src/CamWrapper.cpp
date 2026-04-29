/**
 * @file CamWrapper.cpp
 * @brief Implementation of the DH camera wrapper providing initialization and frame handling.
 */

#include "CamWrapper.h"

#include <glog/logging.h>

#include <chrono>
#include <mutex>
#include <thread>

#include "CamWrapperDH.h"

/**
 * @brief Helper that toggles a failure flag if an SDK call does not succeed.
 * @param status Return status from a `GX` API call.
 * @param flag   Flag that becomes `true` when the call fails (only if fatal=true).
 * @param w_str  Optional tag to help identify the failing parameter in logs.
 * @param fatal  If true, failure is treated as critical and will set flag.
 */
void update_bool(GX_STATUS status, bool& flag, const std::string& w_str, bool fatal) {
    if (status == GX_STATUS_SUCCESS) return;

    if (fatal) {
        flag = true;
        LOG(ERROR) << w_str << " set failed (fatal). status=" << status;
    } else {
        LOG(WARNING) << w_str << " set failed (non-fatal, ignored). status=" << status;
    }
}

// critical params (must success!)
#define UPDB_FATAL(x, wstr) (update_bool(x, set_failed, wstr, true))
// non-critical params (try best effort)
#define UPDB_SOFT(x, wstr) (update_bool(x, set_failed, wstr, false))

/**
 * @brief Convert raw camera buffers into RGB frames matching the ROI settings.
 * @param pImageBuf           Input buffer received from the camera.
 * @param pImageRaw8Buf       Scratch buffer used for 8-bit conversions.
 * @param pImageRGBBuf        Output buffer for the RGB data.
 * @param nImageWidth         Width of the source frame.
 * @param nImageHeight        Height of the source frame.
 * @param nPixelFormat        Pixel format reported by the camera.
 * @param nPixelColorFilter   Color filter identifier used during debayering.
 */
void ProcessData(void* pImageBuf, void* pImageRaw8Buf, void* pImageRGBBuf, int nImageWidth,
                 int nImageHeight, int nPixelFormat, int nPixelColorFilter) {
    switch (nPixelFormat) {
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:

            DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);

            DxRaw8toRGB24Ex(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight,
                            RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(nPixelColorFilter),
                            false, DX_ORDER_BGR);
            break;

        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:

            DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_2_9);

            DxRaw8toRGB24Ex(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight,
                            RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(nPixelColorFilter),
                            false, DX_ORDER_BGR);
            break;

        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:

            DxRaw8toRGB24Ex(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight,
                            RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(nPixelColorFilter),
                            false, DX_ORDER_BGR);
            break;

        case GX_PIXEL_FORMAT_MONO12:

            DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);

            DxRaw8toRGB24Ex(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight,
                            RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(NONE),
                            false, DX_ORDER_BGR);
            break;

        case GX_PIXEL_FORMAT_MONO10:

            DxRaw16toRaw8(pImageBuf, pImageRaw8Buf, nImageWidth, nImageHeight, DX_BIT_4_11);

            DxRaw8toRGB24Ex(pImageRaw8Buf, pImageRGBBuf, nImageWidth, nImageHeight,
                            RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(NONE),
                            false, DX_ORDER_BGR);
            break;

        case GX_PIXEL_FORMAT_MONO8:

            DxRaw8toRGB24Ex(pImageBuf, pImageRGBBuf, nImageWidth, nImageHeight,
                            RAW2RGB_NEIGHBOUR, DX_PIXEL_COLOR_FILTER(NONE),
                            false, DX_ORDER_BGR);
            break;

        default:
            break;
    }
}

/**
 * @brief Callback executed by the GxSDK for every captured frame.
 * @param pFrame Pointer describing the completed frame.
 */
void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame) {
    if (pFrame->status != GX_FRAME_STATUS_SUCCESS) return;

    DHCamera* cam = (DHCamera*)pFrame->pUserParam;
    auto start = std::chrono::steady_clock::now();

    ProcessData((void*)pFrame->pImgBuf, cam->g_pRaw8Buffer, cam->p_buffer_.data(),
                pFrame->nWidth, pFrame->nHeight, pFrame->nPixelFormat,
                cam->g_nColorFilter);

    // Move the filled buffer out; re-allocate for the next frame
    std::vector<uint8_t> frame_data = std::move(cam->p_buffer_);
    cam->p_buffer_.resize(static_cast<size_t>(cam->_roi_h) * cam->_roi_w * 3);

    auto end = std::chrono::steady_clock::now();
    cam->frame_cnt++;
    cam->frame_get_time +=
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    if (cam->frame_cnt == 500) {
        double fps_time_interval =
            std::chrono::duration_cast<std::chrono::milliseconds>(end - cam->fps_time_point)
                .count();
        LOG(INFO) << "average hkcamera delay(ms):" << cam->frame_get_time / cam->frame_cnt
                  << " acq fps:" << 1000.0 / (fps_time_interval / 500.0);
        cam->frame_get_time = cam->frame_cnt = 0;
        cam->fps_time_point = end;
    }

    if (cam->frame_callback_) {
        cam->frame_callback_(std::move(frame_data), pFrame->nTimestamp);
    }
}

/**
 * @brief Polling loop that dequeues frames and updates the cached `cv::Mat` buffers.
 * @param cam Active `DHCamera` instance associated with the acquisition thread.
 */
void getRGBImage(DHCamera* cam) {
    while (1) {
        if (!cam->thread_running) {
            return;
        }
        GX_STATUS status;
        status = GXDQBuf(cam->g_hDevice, &cam->g_frameBuffer, 1000);
        auto start = std::chrono::steady_clock::now();
        // cam->g_frameBuffer.

        cam->pimg_lock.lock();
        ProcessData(cam->g_frameBuffer->pImgBuf, cam->g_pRaw8Buffer, cam->p_buffer_.data(),
                    cam->g_frameBuffer->nWidth, cam->g_frameBuffer->nHeight,
                    cam->g_frameBuffer->nPixelFormat, cam->g_nColorFilter);
        cam->pimg_lock.unlock();
        GXQBuf(cam->g_hDevice, cam->g_frameBuffer);
        auto end = std::chrono::steady_clock::now();
        cam->frame_cnt++;
        cam->frame_get_time +=
            std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        if (cam->frame_cnt == 500) {
            std::cout << "average camera delay(ms):" << cam->frame_get_time / cam->frame_cnt
                      << std::endl;
            cam->frame_get_time = cam->frame_cnt = 0;
        }
    }
}

/**
 * @brief Construct a DH camera wrapper.
 * @param sn     Serial number used to select the physical device.
 * @param roi_w  Initial ROI width.
 * @param rio_h  Initial ROI height.
 */
DHCamera::DHCamera(std::string sn, int roi_w, int rio_h)
    : sn(sn),
      _roi_w(roi_w),
      _roi_h(rio_h),
      frame_cnt(0),
      frame_get_time(0),
      thread_running(false),
      init_success(false) {
    p_buffer_.resize(static_cast<size_t>(_roi_h) * _roi_w * 3);
};

/**
 * @brief Release SDK resources and stop the acquisition if still active.
 */
DHCamera::~DHCamera() {
    if (init_success) {
        stop();
        if (g_frameData.pImgBuf != NULL) {
            free(g_frameData.pImgBuf);
        }
        GXCloseDevice(g_hDevice);
    }
    GXCloseLib();
}

std::string gc_device_typename[5] = {"GX_DEVICE_CLASS_UNKNOWN", "GX_DEVICE_CLASS_USB2",
                                     "GX_DEVICE_CLASS_GEV", "GX_DEVICE_CLASS_U3V",
                                     "GX_DEVICE_CLASS_SMART"};

/**
 * @brief Initialize the camera device with the supplied ROI and exposure configuration.
 * @param roi_x    ROI origin (X) within the sensor.
 * @param roi_y    ROI origin (Y) within the sensor.
 * @param roi_w    ROI width.
 * @param roi_h    ROI height.
 * @param exposure Exposure time in microseconds.
 * @param gain     Gain value for the sensor amplifier.
 * @param FPS      Target acquisition frame rate.
 * @param nBinning Binning factor applied in both axes.
 * @return `true` if all SDK calls succeeded and the camera is ready.
 */
bool DHCamera::init(int roi_x, int roi_y, int roi_w, int roi_h, float exposure, float gain,
                    float FPS, int nBinning, int rotate_type) {
    GXInitLib();
    GXUpdateDeviceList(&nDeviceNum, 1000);
    if (nDeviceNum >= 1) {
        GX_DEVICE_BASE_INFO pBaseinfo[nDeviceNum];
        size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
        status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
        bool found_device = false;
        std::string model_name;

        for (int i = 0; i < nDeviceNum; ++i) {
            std::cout << "device: SN:" << pBaseinfo[i].szSN
                      << " NAME:" << pBaseinfo[i].szDisplayName
                      << " TYPE:" << gc_device_typename[pBaseinfo[i].deviceClass] << std::endl;

            if (std::string(pBaseinfo[i].szSN) == sn) {
                found_device = true;
                model_name = pBaseinfo[i].szDisplayName;
            }
        }

        if (!found_device) {
            std::cerr << "No device found with SN:" << sn << std::endl;
            return false;
        }

        bool strict_param_check = true;

        // WARNING: the model MER-131 is special (old), it can not config the fps ... in the current
        // node, so we disable the strict_param_check for it
        if (model_name.find("MER-131") != std::string::npos) {
            strict_param_check = false;
        }
        // if there is more models that need loose parameter setting, add here
        // if (model_name.find("YOUR_MODEL") != std::string::npos) {
        //     strict_param_check = false;
        // }

        LOG(INFO) << "Using camera model: " << model_name
                  << ", strict_param_check=" << (strict_param_check ? "true" : "false");

        GX_OPEN_PARAM stOpenParam;
        stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
        stOpenParam.openMode = GX_OPEN_SN;
        stOpenParam.pszContent = const_cast<char*>(sn.c_str());
        status = GXOpenDevice(&stOpenParam, &g_hDevice);

        GXGetInt(g_hDevice, GX_INT_SENSOR_WIDTH, &g_SensorWidth);
        GXGetInt(g_hDevice, GX_INT_SENSOR_HEIGHT, &g_SensorHeight);
        std::cout << "DHCamera Sensor: " << g_SensorWidth << " X " << g_SensorHeight << std::endl;

        bool set_failed = false;

        // Binning: MER-131 does not support it through
        if (strict_param_check) {
            // current cam: the init must success, else raise failed
            UPDB_FATAL(GXSetEnum(g_hDevice, GX_ENUM_BINNING_HORIZONTAL_MODE,
                                 GX_BINNING_HORIZONTAL_MODE_AVERAGE),
                       "Binning_h_mode");
            UPDB_FATAL(GXSetEnum(g_hDevice, GX_ENUM_BINNING_VERTICAL_MODE,
                                 GX_BINNING_VERTICAL_MODE_AVERAGE),
                       "Binning_v_mode");
        } else {
            // MER-131 like: non-critical fail
            UPDB_SOFT(GXSetEnum(g_hDevice, GX_ENUM_BINNING_HORIZONTAL_MODE,
                                GX_BINNING_HORIZONTAL_MODE_AVERAGE),
                      "Binning_h_mode");
            UPDB_SOFT(GXSetEnum(g_hDevice, GX_ENUM_BINNING_VERTICAL_MODE,
                                GX_BINNING_VERTICAL_MODE_AVERAGE),
                      "Binning_v_mode");
        }

        GXSetInt(g_hDevice, GX_INT_BINNING_HORIZONTAL, nBinning);
        GXSetInt(g_hDevice, GX_INT_BINNING_VERTICAL, nBinning);

        // ROI, gain, exposure must success, else raise failed for ALL MODELS
        UPDB_FATAL(GXSetInt(g_hDevice, GX_INT_OFFSET_X, roi_x), "ROI_X");
        UPDB_FATAL(GXSetInt(g_hDevice, GX_INT_OFFSET_Y, roi_y), "ROI_Y");
        UPDB_FATAL(GXSetInt(g_hDevice, GX_INT_WIDTH, roi_w), "ROI_W");
        UPDB_FATAL(GXSetInt(g_hDevice, GX_INT_HEIGHT, roi_h), "ROI_H");

        UPDB_FATAL(GXSetEnum(g_hDevice, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF),
                   "ExposureAuto");
        UPDB_FATAL(GXSetEnum(g_hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF), "GainAuto");
        UPDB_FATAL(
            GXSetEnum(g_hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS),
            "BalanceWhiteAuto");

        UPDB_FATAL(GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS),
                   "AcquisitionMode");

        UPDB_FATAL(GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure), "Exposure");
        UPDB_FATAL(GXSetFloat(g_hDevice, GX_FLOAT_GAIN, gain), "Gain");

        // MER-131 does not support fps setting, so we treat it as non-critical fail
        if (strict_param_check) {
            UPDB_FATAL(GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                                 GX_ACQUISITION_FRAME_RATE_MODE_ON),
                       "FPS_Ctrl");
            UPDB_FATAL(GXSetFloat(g_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, FPS), "FPS");
        } else {
            UPDB_SOFT(GXSetEnum(g_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
                                GX_ACQUISITION_FRAME_RATE_MODE_ON),
                      "FPS_Ctrl");
            UPDB_SOFT(GXSetFloat(g_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, FPS), "FPS");
        }

        // gain selector set to ALL
        UPDB_FATAL(GXSetEnum(g_hDevice, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL),
                   "GainSelector");

        // Hardware image flip for 180° rotation (zero CPU cost). Always write both flags so
        // a previous run cannot leave the camera reversed after rotation is disabled.
        const bool enable_180_flip = rotate_type == 1;
        UPDB_SOFT(GXSetBool(g_hDevice, GX_BOOL_REVERSE_X, enable_180_flip), "ReverseX");
        UPDB_SOFT(GXSetBool(g_hDevice, GX_BOOL_REVERSE_Y, enable_180_flip), "ReverseY");
        if (enable_180_flip) {
            LOG(INFO) << "Hardware 180° flip enabled (ReverseX + ReverseY)";
        }

        if (set_failed) {
            LOG(ERROR) << "failed to set some critical parameters!";
            return false;
        }

        GX_FLOAT_RANGE gainRange;
        GXGetFloatRange(g_hDevice, GX_FLOAT_GAIN, &gainRange);
        std::cout << "DHCamera Gain Range: " << gainRange.dMin << "~" << gainRange.dMax
                  << " step size:" << gainRange.dInc << std::endl;
        GXGetInt(g_hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);

        g_frameData.pImgBuf = malloc(nPayLoadSize);

        GXGetEnum(g_hDevice, GX_ENUM_PIXEL_FORMAT, &g_nPixelFormat);
        GXGetEnum(g_hDevice, GX_ENUM_PIXEL_COLOR_FILTER, &g_nColorFilter);

        // Query hardware timestamp tick frequency for accurate frame timing
        if (GXGetInt(g_hDevice, GX_INT_TIMESTAMP_TICK_FREQUENCY, &timestamp_tick_frequency_) != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "Failed to get timestamp tick frequency, hw timestamps won't be used";
            timestamp_tick_frequency_ = 0;
        } else {
            LOG(INFO) << "Camera timestamp tick frequency: " << timestamp_tick_frequency_ << " Hz";
        }

        init_success = true;
        thread_running = false;
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Update exposure and gain while the device is already initialized.
 * @param exposure Exposure time in microseconds.
 * @param gain     Gain value for the sensor amplifier.
 */
void DHCamera::setParam(float exposure, float gain) {
    if (init_success) {
        GXSetFloat(g_hDevice, GX_FLOAT_EXPOSURE_TIME, exposure);
        GXSetFloat(g_hDevice, GX_FLOAT_GAIN, gain);
    }
}

/**
 * @brief Start the asynchronous acquisition using the SDK callback.
 * @return `true` if the start command is accepted.
 */
bool DHCamera::start() {
    if (init_success) {
        frame_cnt = frame_get_time = 0;
        fps_time_point = std::chrono::steady_clock::now();
        GXRegisterCaptureCallback(g_hDevice, this, OnFrameCallbackFun);
        GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_START);
        return true;
    }
    return false;
}

/**
 * @brief Stop acquisition and unregister the frame callback.
 */
void DHCamera::stop() {
    if (init_success) {
        GXSendCommand(g_hDevice, GX_COMMAND_ACQUISITION_STOP);
        GXUnregisterCaptureCallback(g_hDevice);
    }
}

/**
 * @brief Report whether the SDK initialization succeeded.
 * @return True when `init` has completed without errors.
 */
bool DHCamera::init_is_successful() { return init_success; }

/**
 * @brief Register a callback invoked on every acquired frame.
 * @param cb Callback receiving the BGR frame data via move.
 */
void DHCamera::set_frame_callback(FrameCallback cb) {
    frame_callback_ = std::move(cb);
}
