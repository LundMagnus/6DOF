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
} // namespace

int main() {
    std::cout << "Process started\n";
    std::cout << "PID: " << getpid() << std::endl;

    const std::string i2c_device = "/dev/i2c-1";
    const uint8_t address = 0x40;

    if (!probe_i2c_address(i2c_device, address)) {
        std::cerr << "I2C address 0x40 not visible on " << i2c_device << std::endl;
        return 1;
    }

    PCA9685 pwm(address, i2c_device);
    if (!pwm.open()) {
        std::cerr << "Failed to open PCA9685 on /dev/i2c-1" << std::endl;
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
