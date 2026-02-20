#include <iostream>
#include <string>
#include <unistd.h>   // getpid(), sleep()
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "Libraries/pca9685/PCA9685.h"

#define MS62_SERVO 0
#define DM996 1
#define BASE 0

namespace {
bool probe_i2c_address(const std::string &device, uint8_t address) {
    int fd = ::open(device.c_str(), O_RDWR);
    if (fd < 0) {
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        ::close(fd);
        return false;
    }

    uint8_t reg = 0x00;
    uint8_t value = 0;
    bool ok = (::write(fd, &reg, 1) == 1) && (::read(fd, &value, 1) == 1);

    ::close(fd);
    return ok;
}

void scan_i2c_bus(const std::string &device) {
    int fd = ::open(device.c_str(), O_RDWR);
    if (fd < 0) {
        std::cerr << "Failed to open " << device << " for scanning" << std::endl;
        return;
    }

    std::cout << "Scanning " << device << "..." << std::endl;
    bool found_any = false;

    for (uint8_t address = 0x03; address <= 0x77; ++address) {
        if (ioctl(fd, I2C_SLAVE, address) < 0) {
            continue;
        }

        uint8_t reg = 0x00;
        uint8_t value = 0;
        if ((::write(fd, &reg, 1) == 1) && (::read(fd, &value, 1) == 1)) {
            std::cout << "Found device at 0x" << std::hex << static_cast<int>(address) << std::dec << std::endl;
            found_any = true;
        }
    }

    if (!found_any) {
        std::cout << "No I2C devices found" << std::endl;
    }

    ::close(fd);
}
} // namespace



int main() {
    std::cout << "Process started\n";
    std::cout << "PID: " << getpid() << std::endl;

    

    const std::string i2c_device = "/dev/i2c-1";
    const uint8_t address = 0x40; // Default I2C address for PCA9685

    // Connect to the PCA9685 and verify it's present before proceeding
    if (!probe_i2c_address(i2c_device, address)) {
        std::cerr << "I2C address 0x" << std::hex << static_cast<int>(address) << std::dec
                  << " not visible on " << i2c_device << std::endl;
        scan_i2c_bus(i2c_device);
        return 1;
    }

    // Create PCA9685 instance and initialize it
    PCA9685 pwm(address, i2c_device);
    if (!pwm.open()) {
        std::cerr << "Failed to open PCA9685 on " << i2c_device << std::endl;
        return 1;
    }

    // Set PWM frequency to 50 Hz for servo control
    if (!pwm.setPWMFreq(50.0f)) {
        std::cerr << "Failed to set PWM frequency" << std::endl;
        return 1;
    }


    while (true) {

        std::cout << "Setting servo to 0 degrees (0.5 ms pulse)" << std::endl;
        if (!pwm.setServoAngle(BASE, MS62_SERVO, 0)) {
            std::cerr << "Failed to set servo pulse" << std::endl;
            return 1;
        }

        sleep(5);   // sleep for 5 seconds

        std::cout << "Setting servo to 135 degrees (1.5 ms pulse)" << std::endl;
        if (!pwm.setServoAngle(BASE, MS62_SERVO, 135)) {
            std::cerr << "Failed to set servo pulse" << std::endl;
            return 1;
        }

        sleep(5);   // sleep for 5 seconds

        std::cout << "Setting servo to 270 degrees (2.5 ms pulse)" << std::endl;
        if (!pwm.setServoAngle(BASE, MS62_SERVO, 270)) {
            std::cerr << "Failed to set servo pulse" << std::endl;
            return 1;
        }
        
        sleep(5);

        std::cout << "Setting servo to 135 degrees (1.5 ms pulse)" << std::endl;
        if (!pwm.setServoAngle(BASE, MS62_SERVO, 135)) {
            std::cerr << "Failed to set servo pulse" << std::endl;
            return 1;
        }

        sleep(5);
    }
}
