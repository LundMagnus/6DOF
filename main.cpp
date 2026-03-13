#include <iostream>
#include <string>
#include <cmath>
#include <unistd.h>   // getpid(), sleep()
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <chrono>   // Time
#include <stdio.h>
#include <signal.h>

#include "Libraries/PCA9685/PCA9685.h"
#include "Libraries/Controller/Controller.h"
#include "Libraries/Utilities/Utilities.h"

// Servo motor types
#define MS62_SERVO 0
#define DM996 1

// Links
#define BASE 0
#define SHOULDER 1

// Other
#define DEADZONE 5000


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

// Global for signal handler cleanup
static PCA9685* g_pwm = nullptr;
static volatile sig_atomic_t g_running = 1;

void signal_handler(int sig) {
    (void)sig;
    g_running = 0;
}


int main() {
    std::cout << "Process started\n";
    std::cout << "PID: " << getpid() << std::endl;


    //
    // I2C
    //
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
    g_pwm = &pwm;  // Set global pointer for cleanup after loop

    // Register Ctrl+C / termination handlers.
    ::signal(SIGINT, signal_handler);
    ::signal(SIGTERM, signal_handler);


    if (!pwm.open()) {
        std::cerr << "Failed to open PCA9685 on " << i2c_device << std::endl;
        return 1;
    }
    std::cout << "PCA9685 initialized at 50Hz." << std::endl;


    //
    // GAME-CONTROLLER
    //
    Controller c8bitdo;

    if (!c8bitdo.checkController()) {
        return 1;
    }
    sleep(1);


    //
    // PREPARING
    //
    uint16_t targetBaseAngle = 135;
    
    SDL_Event e;

    std::cout << "Ready!" << std::endl;
    

    //
    // PROGRAM START
    //
    while (g_running && c8bitdo.getProgramState()) {
        c8bitdo.updateAxes();

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_JOYBUTTONDOWN || e.type == SDL_CONTROLLERBUTTONDOWN) {
                c8bitdo.handleJoyButtons(e);
            }

            if (e.type == SDL_QUIT || e.type == SDL_JOYDEVICEREMOVED) {
                g_running = 0;
            }
        }
        //std::cout << c8bitdo.getProgramState() << std::endl;

        const int16_t lsx = c8bitdo.getLSX();
        const int16_t lsy = c8bitdo.getLSY();
        if (std::abs(lsx) > DEADZONE || std::abs(lsy) > DEADZONE) {
            float angleLS = constrain(c8bitdo.getLSAngle(), 0, 270);
            targetBaseAngle = angleLS;
        }

        //pwm.setSmoothServoAngle(BASE, MS62_SERVO, targetBaseAngle, 2);
        pwm.setSmoothServoAngle(SHOULDER, MS62_SERVO, targetBaseAngle, 2);
        //pwm.setServoPulse(SHOULDER, targetBaseAngle);
        usleep(100000);
    }

    // Program stopping
    std::cout << "Goodbye." << std::endl;
    for (int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 4096);   // Turn off channel (full-off via 4096)
    }

    pwm.sleep();                 // Put PCA9685 to sleep to stop all outputs

    return 0;

}
