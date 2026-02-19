#include <iostream>
#include <string>
#include <unistd.h>   // getpid(), sleep()
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#include "Libraries/pca9685/PCA9685.h"

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
    const uint8_t address = 0x40;

    scan_i2c_bus(i2c_device);

    if (!probe_i2c_address(i2c_device, address)) {
        std::cerr << "I2C address 0x" << std::hex << static_cast<int>(address) << std::dec
                  << " not visible on " << i2c_device << std::endl;
        scan_i2c_bus(i2c_device);
        return 1;
    }

    PCA9685 pwm(address, i2c_device);
    if (!pwm.open()) {
        std::cerr << "Failed to open PCA9685 on " << i2c_device << std::endl;
        return 1;
    }

    if (!pwm.setPWMFreq(50.0f)) {
        std::cerr << "Failed to set PWM frequency" << std::endl;
        return 1;
    }

    int counter = 0;
    while (true) {
        std::cout << "Tick " << counter++ << std::endl;
        sleep(1);   // sleep for 1 second
    }
}
