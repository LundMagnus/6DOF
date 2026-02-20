#include "PCA9685.h"

#include <iostream>
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
#define MS62_SERVO 0     // 25kg servo motor
constexpr float MS62_MIN_PULSE_MS = 0.5f;
constexpr float MS62_MAX_PULSE_MS = 2.5f;
constexpr uint16_t MS62_MAX_ANGLE = 270;
#define DM996_SERVO 1   // 15kg servo motor
constexpr float DM996_MIN_PULSE_MS = 0.5f;
constexpr float DM996_MAX_PULSE_MS = 2.5f;
constexpr uint16_t DM996_MAX_ANGLE = 180;
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

    // Wake up and enable auto-increment (AI bit = 0x20)
    uint8_t newmode = static_cast<uint8_t>((mode1 & ~MODE1_SLEEP) | 0x20);
    if (!write8(MODE1, newmode)) {
        close();
        return false;
    }

    // Set output mode to totem-pole (not open-drain)
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

    // Enter sleep mode to set prescaler
    uint8_t sleep = static_cast<uint8_t>((oldmode & 0x7F) | MODE1_SLEEP);
    if (!write8(MODE1, sleep)) {
        return false;
    }

    // Set prescaler
    if (!write8(PRESCALE, prescale)) {
        return false;
    }

    // Restore previous mode and restart
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

float map(double x, double fromLow, double fromHigh, double toLow, double toHigh) {
  return toLow + (x-fromLow)*(toHigh-toLow)/(fromHigh-fromLow);
}

float constrain(double x, double a, double b) {
    if(x < a) {
        return a;
    } else if(x > b) {
        return b;
    } else {
        return x;
    }
}

bool PCA9685::setServoAngle(uint8_t channel, uint8_t servoType, uint16_t servoAngle) {
    float val = 0.0f;

    // Use appropiate servo angle calculation
    switch(servoType) {
        case MS62_SERVO:
            val = constrain(map(servoAngle, 0, MS62_MAX_ANGLE, MS62_MIN_PULSE_MS, MS62_MAX_PULSE_MS), MS62_MIN_PULSE_MS, MS62_MAX_PULSE_MS);
            break;
        case DM996_SERVO:
            val = constrain(map(servoAngle, 0, DM996_MAX_ANGLE, DM996_MIN_PULSE_MS, DM996_MAX_PULSE_MS), DM996_MIN_PULSE_MS, DM996_MAX_PULSE_MS);
            break;
        default:
            std::cerr << "Incorrect servo-type index was inputted." << std::endl;
            return false;
    }

    std::cout << "Channel " << static_cast<int>(channel) << ": angle=" << servoAngle 
              << "° -> pulse=" << val << "ms" << std::endl;
    return setServoPulse(channel, val);
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
