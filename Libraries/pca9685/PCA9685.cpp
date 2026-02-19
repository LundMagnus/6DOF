#include "PCA9685.h"

#include <cmath>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

namespace {
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t MODE2 = 0x01;
constexpr uint8_t PRESCALE = 0xFE;
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t MODE1_SLEEP = 0x10;
constexpr uint8_t MODE1_RESTART = 0x80;
constexpr uint8_t MODE2_OUTDRV = 0x04;
constexpr float OSC_CLOCK_HZ = 25000000.0f;
constexpr uint16_t RESOLUTION = 4096;
} // namespace

PCA9685::PCA9685(uint8_t address, std::string i2c_device)
    : fd_(-1), address_(address), i2c_device_(std::move(i2c_device)), current_freq_hz_(50.0f) {}

PCA9685::~PCA9685() {
    close();
}

bool PCA9685::open() {
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(i2c_device_.c_str(), O_RDWR);
    if (fd_ < 0) {
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    uint8_t mode1 = 0;
    if (!read8(MODE1, mode1)) {
        close();
        return false;
    }

    if (!write8(MODE1, static_cast<uint8_t>(mode1 & ~MODE1_SLEEP))) {
        close();
        return false;
    }

    if (!write8(MODE2, MODE2_OUTDRV)) {
        close();
        return false;
    }

    return setPWMFreq(current_freq_hz_);
}

void PCA9685::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool PCA9685::setPWMFreq(float freq_hz) {
    if (fd_ < 0) {
        return false;
    }

    float prescaleval = (OSC_CLOCK_HZ / (RESOLUTION * freq_hz)) - 1.0f;
    uint8_t prescale = static_cast<uint8_t>(std::lround(prescaleval));

    uint8_t oldmode = 0;
    if (!read8(MODE1, oldmode)) {
        return false;
    }

    uint8_t sleep = static_cast<uint8_t>((oldmode & 0x7F) | MODE1_SLEEP);
    if (!write8(MODE1, sleep)) {
        return false;
    }

    if (!write8(PRESCALE, prescale)) {
        return false;
    }

    if (!write8(MODE1, oldmode)) {
        return false;
    }

    usleep(5000);
    if (!write8(MODE1, static_cast<uint8_t>(oldmode | MODE1_RESTART))) {
        return false;
    }

    current_freq_hz_ = freq_hz;
    return true;
}

bool PCA9685::setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    if (fd_ < 0 || channel >= 16) {
        return false;
    }

    uint8_t data[4] = {
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>((on >> 8) & 0x0F),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>((off >> 8) & 0x0F)
    };

    uint8_t reg = static_cast<uint8_t>(LED0_ON_L + 4 * channel);
    return writeBlock(reg, data, sizeof(data));
}

bool PCA9685::setServoPulse(uint8_t channel, float pulse_ms) {
    float period_ms = 1000.0f / current_freq_hz_;
    float ticks = (pulse_ms / period_ms) * RESOLUTION;

    if (ticks < 0.0f) {
        ticks = 0.0f;
    }

    if (ticks > (RESOLUTION - 1)) {
        ticks = RESOLUTION - 1;
    }

    return setPWM(channel, 0, static_cast<uint16_t>(std::lround(ticks)));
}

bool PCA9685::write8(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return ::write(fd_, buffer, sizeof(buffer)) == static_cast<ssize_t>(sizeof(buffer));
}

bool PCA9685::writeBlock(uint8_t reg, const uint8_t *data, size_t len) {
    if (!data || len == 0) {
        return false;
    }

    std::vector<uint8_t> buffer(len + 1);
    buffer[0] = reg;
    std::memcpy(buffer.data() + 1, data, len);
    return ::write(fd_, buffer.data(), buffer.size()) == static_cast<ssize_t>(buffer.size());
}

bool PCA9685::read8(uint8_t reg, uint8_t &value) {
    if (::write(fd_, &reg, 1) != 1) {
        return false;
    }

    if (::read(fd_, &value, 1) != 1) {
        return false;
    }

    return true;
}
