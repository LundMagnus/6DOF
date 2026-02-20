#pragma once

#include <cstdint>
#include <string>

class PCA9685 {
public:
    explicit PCA9685(uint8_t address = 0x40, std::string i2c_device = "/dev/i2c-1");
    ~PCA9685();

    bool open();
    void close();

    bool setPWMFreq(float freq_hz);
    bool setPWM(uint8_t channel, uint16_t on, uint16_t off);
    bool setServoPulse(uint8_t channel, float pulse_ms);
    bool setServoAngle(uint8_t channel, uint8_t servoType, uint8_t servoAngle);

private:
    bool write8(uint8_t reg, uint8_t value);
    bool writeBlock(uint8_t reg, const uint8_t *data, size_t len);
    bool read8(uint8_t reg, uint8_t &value);

    int fd_;
    uint8_t address_;
    std::string i2c_device_;
    float current_freq_hz_;
};
