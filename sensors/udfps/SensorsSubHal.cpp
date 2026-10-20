// SPDX-License-Identifier: Apache-2.0

#include "SensorsSubHal.h"

#include <fcntl.h>
#include <log/log.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

using ::android::hardware::sensors::V2_1::implementation::ISensorsSubHal;

namespace android {
namespace hardware {
namespace sensors {
namespace V2_1 {
namespace subhal {
namespace implementation {

namespace {

constexpr char kFodStatusPath[] = "/sys/class/touch/touch_dev/fod_press_status";
constexpr char kTouchDevicePath[] = "/dev/xiaomi-touch";
constexpr int kPrimaryTouchId = 0;
constexpr uint16_t kTouchFodEnable = 10;
constexpr int kTouchSelectIdCommand = 3;
constexpr int kTouchCommonDataCommand = 0;
constexpr int kTouchIoctlMagic = 0x54;
constexpr size_t kTouchDataBufferSize = 256;

struct TouchControl {
    int8_t touchId;
    uint8_t command;
    uint16_t mode;
    uint16_t dataLength;
    int32_t data[kTouchDataBufferSize];
};

static_assert(sizeof(TouchControl) == 1032, "Unexpected Klee touch ioctl structure size");

constexpr unsigned long kSelectTouchId =
        _IOW(kTouchIoctlMagic, kTouchSelectIdCommand, int);
constexpr unsigned long kCommonData =
        _IOW(kTouchIoctlMagic, kTouchCommonDataCommand, TouchControl);

class KleeUdfpsSensor final : public SysfsPollingOneShotSensor {
  public:
    KleeUdfpsSensor(int32_t sensorHandle, ISensorsEventCallback* callback)
        : SysfsPollingOneShotSensor(
                  sensorHandle, callback, kFodStatusPath, "", "UDFPS Sensor",
                  "org.lineageos.sensor.udfps",
                  static_cast<SensorType>(static_cast<int32_t>(SensorType::DEVICE_PRIVATE_BASE) +
                                          3)) {}

    void writeEnable(bool enable) override {
        const int fd = open(kTouchDevicePath, O_RDWR | O_CLOEXEC);
        if (fd < 0) {
            ALOGE("Failed to open %s: %s", kTouchDevicePath, strerror(errno));
            return;
        }

        if (ioctl(fd, kSelectTouchId, kPrimaryTouchId) < 0) {
            ALOGE("Failed to select primary touch for FOD: %s", strerror(errno));
            close(fd);
            return;
        }

        // Match the payload used by UdfpsHandler::setFodStatus().
        TouchControl request{};
        request.touchId = kPrimaryTouchId;
        request.command = 0;  // SET_CUR_VALUE
        request.mode = kTouchFodEnable;
        request.data[0] = enable ? 1 : 0;

        if (ioctl(fd, kCommonData, &request) < 0) {
            ALOGE("Failed to %s touch FOD: %s", enable ? "enable" : "disable",
                  strerror(errno));
        }
        close(fd);
    }

    bool readFd(const int fd) override {
        char buffer[512] = {};
        if (lseek(fd, 0, SEEK_SET) < 0) {
            ALOGE("Failed to seek FOD status: %s", strerror(errno));
            return false;
        }

        const ssize_t length = read(fd, buffer, sizeof(buffer) - 1);
        if (length <= 0) {
            ALOGE("Failed to read FOD status: %s", strerror(errno));
            return false;
        }
        buffer[length] = '\0';

        int state = 0;
        const int fields = sscanf(buffer, "%d,%d,%d", &mScreenX, &mScreenY, &state);
        if (fields == 1) {
            // Some Klee firmware reports only the pressed state.
            state = mScreenX;
            mScreenX = 0;
            mScreenY = 0;
        } else if (fields < 3) {
            ALOGE("Could not parse FOD status: %s", buffer);
            return false;
        }

        return state > 0;
    }

    void fillEventData(Event& event) override {
        event.u.data[0] = mScreenX;
        event.u.data[1] = mScreenY;
    }

  private:
    int mScreenX = 0;
    int mScreenY = 0;
};

}  // namespace

class KleeSensorsSubHal final : public SensorsSubHal {
  public:
    KleeSensorsSubHal() {
        AddSensor<KleeUdfpsSensor>();
    }

    const std::string getName() override {
        return "Klee UDFPS";
    }
};

}  // namespace implementation
}  // namespace subhal
}  // namespace V2_1
}  // namespace sensors
}  // namespace hardware
}  // namespace android

ISensorsSubHal* sensorsHalGetSubHal_2_1(uint32_t* version) {
    static android::hardware::sensors::V2_1::subhal::implementation::
            KleeSensorsSubHal subHal;
    *version = SUB_HAL_2_1_VERSION;
    return &subHal;
}
