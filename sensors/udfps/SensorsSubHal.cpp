// SPDX-License-Identifier: Apache-2.0

#include "SensorsSubHal.h"

#include <cutils/properties.h>
#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <log/log.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <utils/SystemClock.h>

#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using ::android::hardware::sensors::V1_0::SensorFlagBits;
using ::android::hardware::sensors::V2_1::implementation::ISensorsSubHal;

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

namespace {

constexpr char kInputDirectory[] = "/dev/input";
constexpr char kTouchInputName[] = "NVTCapacitiveTouchScreen";
constexpr char kTouchDevicePath[] = "/dev/xiaomi-touch";
constexpr int kPrimaryTouchId = 0;
// The Klee NT38771 driver reports screen-off FOD on reserved MT slot 9.
constexpr int kFodGestureSlot = 9;
constexpr uint16_t kTouchFodEnable = 10;
constexpr uint16_t kTouchFodIconEnable = 16;
// NVT only arms its screen-off FOD IRQ when touch mode 35 is set to 5.
constexpr uint16_t kTouchFodWakeState = 35;
constexpr int32_t kScreenOffFodWakeState = 5;
constexpr uint8_t kTouchGetCurrentValue = 1;
constexpr int kTouchSelectIdCommand = 3;
constexpr int kTouchCommonDataCommand = 0;
constexpr int kTouchIoctlMagic = 0x54;
constexpr size_t kTouchDataBufferSize = 256;
constexpr float kDefaultTouchCoordinateScale = 100.0f;
constexpr float kDefaultFodCenterX = 638.0f;
constexpr float kDefaultFodCenterY = 2516.0f;
constexpr float kDefaultFodRadius = 105.0f;
constexpr float kFodRadiusTolerance = 24.0f;

struct TouchControl {
    int8_t touchId;
    uint8_t command;
    uint16_t mode;
    uint16_t dataLength;
    int32_t data[kTouchDataBufferSize];
};

static_assert(sizeof(TouchControl) == 1032, "Unexpected Klee touch ioctl structure size");

constexpr unsigned long kSelectTouchId = _IOW(kTouchIoctlMagic, kTouchSelectIdCommand, int);
constexpr unsigned long kCommonData = _IOW(kTouchIoctlMagic, kTouchCommonDataCommand, TouchControl);
constexpr unsigned long kGetCommonData =
        _IOWR(kTouchIoctlMagic, kTouchCommonDataCommand, TouchControl);

class KleeUdfpsSensor final : public OneShotSensor {
  public:
    KleeUdfpsSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
        : OneShotSensor(sensorHandle, callback) {
        mSensorInfo.name = "UDFPS Sensor";
        mSensorInfo.vendor = "Xiaomi";
        mSensorInfo.type =
                static_cast<SensorType>(static_cast<int32_t>(SensorType::DEVICE_PRIVATE_BASE) + 3);
        mSensorInfo.typeAsString = "org.lineageos.sensor.udfps";
        mSensorInfo.maxRange = 4096.0f;
        mSensorInfo.resolution = 1.0f;
        mSensorInfo.power = 0.001f;
        mSensorInfo.minDelay = -1;
        mSensorInfo.maxDelay = 0;
        mSensorInfo.flags |= SensorFlagBits::WAKE_UP;

        readFodLocation();

        if (pipe2(mWakePipeFd, O_CLOEXEC | O_NONBLOCK) < 0) {
            ALOGE("Failed to create UDFPS input wake pipe: %s", strerror(errno));
            mWakePipeFd[0] = -1;
            mWakePipeFd[1] = -1;
        }

        mInputFd = findTouchInputDevice();
        if (mInputFd >= 0) {
            if (enableScreenOffFodModes()) {
                ALOGI("Pre-armed NVT screen-off FOD modes before touchscreen suspend");
            } else {
                ALOGE("Could not pre-arm NVT screen-off FOD modes");
            }
        }
    }

    ~KleeUdfpsSensor() override {
        bool restoreTouchModes = false;
        {
            std::lock_guard<std::mutex> lock(mStateMutex);
            mStopped = true;
            mActive = false;
            restoreTouchModes = mTouchModesEnabled || mTouchModesDirty ||
                                mPreviousFodIconStateSaved || mPreviousFodWakeStateSaved;
            mTouchModesEnabled = false;
            mTouchModesDirty = false;
        }

        if (restoreTouchModes) {
            setTouchMode(kTouchFodEnable, "FOD", 0);
            if (!restoreScreenOffFodModes()) {
                ALOGE("Failed to restore NVT FOD modes while stopping the sensor");
            }
        }
        wakeInputThread();
        if (mInputThread.joinable()) {
            mInputThread.join();
        }

        if (mInputFd >= 0) {
            close(mInputFd);
        }
        if (mWakePipeFd[0] >= 0) {
            close(mWakePipeFd[0]);
        }
        if (mWakePipeFd[1] >= 0) {
            close(mWakePipeFd[1]);
        }
    }

    void activate(bool enable) override {
        std::lock_guard<std::mutex> lock(mStateMutex);
        if (mStopped) {
            return;
        }

        if (enable) {
            if (mActive) {
                return;
            }
            if (mWakePipeFd[0] < 0 || mWakePipeFd[1] < 0) {
                ALOGE("UDFPS input sensor is unavailable: wake pipe creation failed");
                return;
            }
            if (mInputFd < 0) {
                mInputFd = findTouchInputDevice();
            }
            if (mInputFd < 0) {
                ALOGE("UDFPS input sensor is unavailable: %s was not found", kTouchInputName);
                return;
            }

            if (!mInputThread.joinable()) {
                mInputThread = std::thread(&KleeUdfpsSensor::inputLoop, this);
            }

            drainInputQueueLocked();
            resetTouchStateLocked();
            mTriggered = false;
            mActive = true;
            mTouchModesDirty = true;
            if (synchronizeTouchModesLocked()) {
                ALOGI("Armed screen-off UDFPS input sensor on FOD slot %d at %.0f,%.0f",
                      kFodGestureSlot, mFodCenterX, mFodCenterY);
            }
        } else {
            if (!mActive && !mTouchModesEnabled && !mTouchModesDirty) {
                return;
            }

            mActive = false;
            resetTouchStateLocked();
            mTouchModesDirty = true;
            synchronizeTouchModesLocked();
        }

        wakeInputThread();
    }

    void setOperationMode(OperationMode mode) override {
        Sensor::setOperationMode(mode);
        std::lock_guard<std::mutex> lock(mStateMutex);
        mOperationMode = mode;
    }

  private:
    void readFodLocation() {
        char location[PROPERTY_VALUE_MAX] = {};
        if (property_get("persist.vendor.fingerprint.sensor_location", location, "") <= 0) {
            ALOGW("Using built-in Klee FOD location; sensor location property is unavailable");
            return;
        }

        float centerX;
        float centerY;
        float radius;
        if (std::sscanf(location, "%f|%f|%f", &centerX, &centerY, &radius) != 3 ||
            !std::isfinite(centerX) || !std::isfinite(centerY) || !std::isfinite(radius) ||
            centerX <= 0.0f || centerY <= 0.0f || radius <= 0.0f) {
            ALOGE("Ignoring invalid FOD location property: %s", location);
            return;
        }

        mFodCenterX = centerX;
        mFodCenterY = centerY;
        mFodRadius = radius;
        ALOGI("Using FOD location %.0f,%.0f radius %.0f", centerX, centerY, radius);
    }

    int findTouchInputDevice() {
        DIR* directory = opendir(kInputDirectory);
        if (directory == nullptr) {
            ALOGE("Failed to open %s: %s", kInputDirectory, strerror(errno));
            return -1;
        }

        int inputFd = -1;
        dirent* entry = nullptr;
        while ((entry = readdir(directory)) != nullptr) {
            if (std::strncmp(entry->d_name, "event", 5) != 0) {
                continue;
            }

            const std::string path = std::string(kInputDirectory) + "/" + entry->d_name;
            const int candidateFd = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NONBLOCK);
            if (candidateFd < 0) {
                continue;
            }

            char name[256] = {};
            if (ioctl(candidateFd, EVIOCGNAME(sizeof(name)), name) >= 0 &&
                std::strncmp(name, kTouchInputName, sizeof(kTouchInputName) - 1) == 0) {
                input_absinfo xInfo{};
                input_absinfo yInfo{};
                if (ioctl(candidateFd, EVIOCGABS(ABS_MT_POSITION_X), &xInfo) >= 0 &&
                    xInfo.maximum > 0) {
                    mCoordinateScaleX =
                            xInfo.maximum > 10000 ? kDefaultTouchCoordinateScale : 1.0f;
                }
                if (ioctl(candidateFd, EVIOCGABS(ABS_MT_POSITION_Y), &yInfo) >= 0 &&
                    yInfo.maximum > 0) {
                    mCoordinateScaleY =
                            yInfo.maximum > 10000 ? kDefaultTouchCoordinateScale : 1.0f;
                }
                inputFd = candidateFd;
                ALOGI("Using UDFPS touch input %s (%s), coordinate scale %.0f,%.0f", name,
                      path.c_str(), mCoordinateScaleX, mCoordinateScaleY);
                break;
            }
            close(candidateFd);
        }

        closedir(directory);
        if (inputFd < 0) {
            ALOGE("Could not find the %s input device", kTouchInputName);
        }
        return inputFd;
    }

    bool setTouchMode(uint16_t mode, const char* name, int32_t value) {
        const int touchFd = open(kTouchDevicePath, O_RDWR | O_CLOEXEC);
        if (touchFd < 0) {
            ALOGE("Failed to open %s: %s", kTouchDevicePath, strerror(errno));
            return false;
        }

        if (ioctl(touchFd, kSelectTouchId, kPrimaryTouchId) < 0) {
            ALOGE("Failed to select primary touch for FOD: %s", strerror(errno));
            close(touchFd);
            return false;
        }

        TouchControl request{};
        request.touchId = kPrimaryTouchId;
        request.command = 0;  // SET_CUR_VALUE
        request.mode = mode;
        request.dataLength = 1;
        request.data[0] = value;
        if (ioctl(touchFd, kCommonData, &request) < 0) {
            ALOGE("Failed to set touch %s mode to %d: %s", name, value, strerror(errno));
            close(touchFd);
            return false;
        }

        close(touchFd);
        return true;
    }

    bool getTouchMode(uint16_t mode, const char* name, int32_t* value) {
        if (value == nullptr) {
            return false;
        }

        const int touchFd = open(kTouchDevicePath, O_RDWR | O_CLOEXEC);
        if (touchFd < 0) {
            ALOGE("Failed to open %s to read %s mode: %s", kTouchDevicePath, name,
                  strerror(errno));
            return false;
        }

        if (ioctl(touchFd, kSelectTouchId, kPrimaryTouchId) < 0) {
            ALOGE("Failed to select primary touch to read %s mode: %s", name, strerror(errno));
            close(touchFd);
            return false;
        }

        TouchControl request{};
        request.touchId = kPrimaryTouchId;
        request.command = kTouchGetCurrentValue;  // GET_CUR_VALUE
        request.mode = mode;
        request.dataLength = 1;
        if (ioctl(touchFd, kGetCommonData, &request) < 0) {
            ALOGE("Failed to read touch %s mode: %s", name, strerror(errno));
            close(touchFd);
            return false;
        }

        *value = request.data[0];
        close(touchFd);
        return true;
    }

    bool enableScreenOffFodModes() {
        if (!mPreviousFodIconStateSaved) {
            if (!getTouchMode(kTouchFodIconEnable, "FOD icon", &mPreviousFodIconState)) {
                return false;
            }
            mPreviousFodIconStateSaved = true;
        }

        if (!setTouchMode(kTouchFodIconEnable, "FOD icon", 1)) {
            return false;
        }

        if (!mPreviousFodWakeStateSaved) {
            if (!getTouchMode(kTouchFodWakeState, "FOD wake state",
                              &mPreviousFodWakeState)) {
                restoreScreenOffFodModes();
                return false;
            }
            mPreviousFodWakeStateSaved = true;
        }

        if (!setTouchMode(kTouchFodWakeState, "FOD wake state",
                          kScreenOffFodWakeState)) {
            restoreScreenOffFodModes();
            return false;
        }

        ALOGI("Armed NVT screen-off FOD modes: mode 16 %d -> 1, mode 35 %d -> %d",
              mPreviousFodIconState, mPreviousFodWakeState, kScreenOffFodWakeState);
        return true;
    }

    bool restoreScreenOffFodModes() {
        bool restored = true;
        if (mPreviousFodWakeStateSaved) {
            if (setTouchMode(kTouchFodWakeState, "FOD wake state",
                             mPreviousFodWakeState)) {
                ALOGI("Restored NVT FOD wake state mode 35 to %d", mPreviousFodWakeState);
                mPreviousFodWakeStateSaved = false;
            } else {
                restored = false;
            }
        }

        if (mPreviousFodIconStateSaved) {
            if (setTouchMode(kTouchFodIconEnable, "FOD icon", mPreviousFodIconState)) {
                ALOGI("Restored NVT FOD icon mode 16 to %d", mPreviousFodIconState);
                mPreviousFodIconStateSaved = false;
            } else {
                restored = false;
            }
        }
        return restored;
    }

    bool setTouchModes(bool enable) {
        // Mode 35 selects the NVT screen-off FOD wake path; mode 16 enables
        // the FOD icon gesture, and mode 10 follows the active Doze listener.
        if (enable) {
            if (!enableScreenOffFodModes()) {
                ALOGE("Could not enable NVT screen-off FOD modes");
                return false;
            }

            if (!setTouchMode(kTouchFodEnable, "FOD", 1)) {
                setTouchMode(kTouchFodEnable, "FOD", 0);
                return false;
            }
            return true;
        }

        // Keep modes 16 and 35 armed while the one-shot listener is inactive.
        // The NVT driver samples them in nvt_ts_suspend(), before SystemUI
        // registers this Doze trigger after the display has turned off.
        return setTouchMode(kTouchFodEnable, "FOD", 0);
    }

    bool synchronizeTouchModesLocked() {
        if (!mTouchModesDirty) {
            return true;
        }

        const bool enable = mActive;
        if (!setTouchModes(enable)) {
            ALOGE("Failed to %s NVT touch modes for screen-off UDFPS",
                  enable ? "arm" : "disable FOD listener");
            return false;
        }

        mTouchModesEnabled = enable;
        mTouchModesDirty = false;
        return true;
    }

    bool isInsideFodArea(float x, float y) const {
        const float dx = x - mFodCenterX;
        const float dy = y - mFodCenterY;
        const float radius = mFodRadius + kFodRadiusTolerance;
        return dx * dx + dy * dy <= radius * radius;
    }

    void drainInputQueueLocked() {
        input_event inputEvent{};
        while (true) {
            const ssize_t bytes = read(mInputFd, &inputEvent, sizeof(inputEvent));
            if (bytes == static_cast<ssize_t>(sizeof(inputEvent))) {
                continue;
            }
            if (bytes < 0 && errno == EINTR) {
                continue;
            }
            if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return;
            }
            if (bytes != 0) {
                ALOGW("Could not completely drain UDFPS input queue: %s",
                      bytes < 0 ? strerror(errno) : "short input event");
            }
            return;
        }
    }

    void resetTouchStateLocked() {
        mCurrentSlot = -1;
        mFodTouchDown = false;
        mHaveX = false;
        mHaveY = false;
        mDiscardUntilSynReport = false;
    }

    bool processInputEventLocked(const input_event& inputEvent, Event* event) {
        if (inputEvent.type == EV_SYN && inputEvent.code == SYN_DROPPED) {
            resetTouchStateLocked();
            mDiscardUntilSynReport = true;
            return false;
        }

        if (mDiscardUntilSynReport) {
            if (inputEvent.type == EV_SYN && inputEvent.code == SYN_REPORT) {
                mDiscardUntilSynReport = false;
            }
            return false;
        }

        if (inputEvent.type == EV_ABS) {
            if (inputEvent.code == ABS_MT_SLOT) {
                mCurrentSlot = inputEvent.value;
            } else if (mCurrentSlot == kFodGestureSlot) {
                if (inputEvent.code == ABS_MT_TRACKING_ID) {
                    mFodTouchDown = inputEvent.value >= 0;
                    mHaveX = false;
                    mHaveY = false;
                } else if (inputEvent.code == ABS_MT_POSITION_X) {
                    mRawX = inputEvent.value;
                    mHaveX = true;
                } else if (inputEvent.code == ABS_MT_POSITION_Y) {
                    mRawY = inputEvent.value;
                    mHaveY = true;
                }
            }
            return false;
        }

        if (inputEvent.type != EV_SYN || inputEvent.code != SYN_REPORT || !mActive || mTriggered ||
            !mFodTouchDown || !mHaveX || !mHaveY ||
            mOperationMode == OperationMode::DATA_INJECTION || mRawX <= 0 || mRawY <= 0) {
            return false;
        }

        const float x = static_cast<float>(mRawX) / mCoordinateScaleX;
        const float y = static_cast<float>(mRawY) / mCoordinateScaleY;
        if (!isInsideFodArea(x, y)) {
            return false;
        }

        event->sensorHandle = mSensorInfo.sensorHandle;
        event->sensorType = mSensorInfo.type;
        event->timestamp = ::android::elapsedRealtimeNano();
        event->u.data[0] = x;
        event->u.data[1] = y;
        const int touchSlot = mCurrentSlot;
        mTriggered = true;
        mActive = false;
        resetTouchStateLocked();
        mTouchModesDirty = true;
        synchronizeTouchModesLocked();
        ALOGI("Screen-off FOD touch on slot %d at (%.0f, %.0f)", touchSlot, event->u.data[0],
              event->u.data[1]);
        return true;
    }

    void processInputEvents(int inputFd) {
        std::vector<Event> events;
        {
            std::lock_guard<std::mutex> lock(mStateMutex);
            if (mStopped || inputFd != mInputFd) {
                return;
            }

            input_event inputEvents[32] = {};
            while (true) {
                const ssize_t bytes = read(inputFd, inputEvents, sizeof(inputEvents));
                if (bytes > 0) {
                    if (bytes % static_cast<ssize_t>(sizeof(input_event)) != 0) {
                        ALOGE("Touch input returned a partial event buffer");
                        resetTouchStateLocked();
                        break;
                    }
                    const size_t count = bytes / sizeof(input_event);
                    for (size_t i = 0; i < count; ++i) {
                        Event event{};
                        if (processInputEventLocked(inputEvents[i], &event)) {
                            events.push_back(event);
                        }
                    }
                    continue;
                }
                if (bytes < 0 && errno == EINTR) {
                    continue;
                }
                if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    break;
                }
                if (bytes < 0) {
                    ALOGE("Failed to read UDFPS touch input: %s", strerror(errno));
                }
                break;
            }
        }

        if (!events.empty()) {
            mCallback->postEvents(events, isWakeUpSensor());
        }
    }

    void inputLoop() {
        while (true) {
            int inputFd;
            int timeoutMs;
            {
                std::lock_guard<std::mutex> lock(mStateMutex);
                if (mStopped) {
                    return;
                }
                inputFd = mInputFd;
                timeoutMs = mTouchModesDirty ? 500 : -1;
            }

            pollfd pollFds[2] = {};
            pollFds[0].fd = mWakePipeFd[0];
            pollFds[0].events = POLLIN;
            pollFds[1].fd = inputFd;
            pollFds[1].events = POLLIN;

            const int result = poll(pollFds, 2, timeoutMs);
            if (result < 0) {
                if (errno == EINTR) {
                    continue;
                }
                ALOGE("Failed to poll UDFPS touch input: %s", strerror(errno));
                return;
            }

            if (result == 0) {
                std::lock_guard<std::mutex> lock(mStateMutex);
                if (!mStopped && mTouchModesDirty) {
                    synchronizeTouchModesLocked();
                }
                continue;
            }

            if (pollFds[0].revents & POLLIN) {
                char buffer[32];
                while (read(mWakePipeFd[0], buffer, sizeof(buffer)) > 0) {
                }
            }

            if (pollFds[1].revents & POLLIN) {
                processInputEvents(inputFd);
            } else if (pollFds[1].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                ALOGE("UDFPS touch input poll error: 0x%x", pollFds[1].revents);
                return;
            }
        }
    }

    void wakeInputThread() {
        if (mWakePipeFd[1] < 0) {
            return;
        }
        const char wake = '1';
        if (write(mWakePipeFd[1], &wake, sizeof(wake)) < 0 && errno != EAGAIN &&
            errno != EWOULDBLOCK) {
            ALOGE("Failed to wake UDFPS input thread: %s", strerror(errno));
        }
    }

    int mInputFd = -1;
    int mWakePipeFd[2] = {-1, -1};
    int mCurrentSlot = -1;
    int32_t mRawX = 0;
    int32_t mRawY = 0;
    float mCoordinateScaleX = kDefaultTouchCoordinateScale;
    float mCoordinateScaleY = kDefaultTouchCoordinateScale;
    float mFodCenterX = kDefaultFodCenterX;
    float mFodCenterY = kDefaultFodCenterY;
    float mFodRadius = kDefaultFodRadius;
    bool mActive = false;
    bool mTriggered = false;
    bool mTouchModesEnabled = false;
    bool mTouchModesDirty = false;
    int32_t mPreviousFodIconState = 0;
    bool mPreviousFodIconStateSaved = false;
    int32_t mPreviousFodWakeState = 0;
    bool mPreviousFodWakeStateSaved = false;
    bool mFodTouchDown = false;
    bool mHaveX = false;
    bool mHaveY = false;
    bool mDiscardUntilSynReport = false;
    bool mStopped = false;
    OperationMode mOperationMode = OperationMode::NORMAL;
    std::mutex mStateMutex;
    std::thread mInputThread;
};

}  // namespace

class KleeSensorsSubHal final : public SensorsSubHal {
  public:
    KleeSensorsSubHal() { AddSensor<KleeUdfpsSensor>(); }

    const std::string getName() override { return "Klee UDFPS"; }
};

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android

ISensorsSubHal* sensorsHalGetSubHal_2_1(uint32_t* version) {
    static android::hardware::sensors::V2_1::subhal::implementation::KleeSensorsSubHal subHal;
    *version = SUB_HAL_2_1_VERSION;
    return &subHal;
}
