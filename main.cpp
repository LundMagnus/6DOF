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
#include <vector>
#include <iomanip>

#include "Libraries/PCA9685/PCA9685.h"
#include "Libraries/Controller/Controller.h"
#include "Libraries/Utilities/Utilities.h"
#include "Libraries/Inverse_Kinematics/Inverse_Kinematics.h"

// Servo motor types
#define MS62_SERVO      0
#define DM996_SERVO     1
#define MS62_SERVO_A    2

// Links
#define BASE        0
#define SHOULDER    1
#define UPPER_ARM   2
#define FOREARM     3
#define WIRST       4
#define FINGER      5

// Other
#define DEADZONE 5000

float x = 0.0;
float y = 0.0;
float z = 0.3;


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
    
    //std::vector<double> IK_Solutions = IK_solver();
    //if(IK_Solutions[0] == -1) {
    //    g_running = 0;
    //} else {
//
    //    std::cout << "Solution found:\n";
    //    for (int i = 0; i < 5; i++) {
    //        std::cout << "q" << i << " deg=" << IK_Solutions[i] << "\n";
    //    }
    //}


    //
    // PROGRAM START
    //
    float angleLS = 90;
    float angleRS = 90;
    float RS = 90;
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

        // Gripper
        const int16_t rsy = c8bitdo.getRSY();
        if(std::abs(rsy) > DEADZONE) { 
            angleRS = c8bitdo.getRSAngle();
            RS = constrain(angleRS, 66, 180);
        }

        // X Y movement
        const int16_t lsx = c8bitdo.getLSX();
        const int16_t lsy = c8bitdo.getLSY();
        if (std::abs(lsx) > DEADZONE || std::abs(lsy) > DEADZONE) {     // To prevent small noise
            float vectorLS = c8bitdo.getLSVector() / INT16_MAX;
            angleLS = c8bitdo.getLSAngle();


            x += (cos(angleLS) * vectorLS)/1000;
            y += (sin(angleLS) * vectorLS)/1000;

        }

        // Z movement
        z -= c8bitdo.getLTCurve()/1000;
        z += c8bitdo.getRTCurve()/1000;



        // x,y,z debug
        std::cout << "x: " << std::setw(5) << x << std::setw(5) << "y: " << std::setw(5) << y << std::setw(5) << "z: " << std::setw(5) << z << std::endl;

        // IK solver
        bool solution_found = false;
        std::vector<double> IK_Solutions = IK_solver(x, y, z);
        if(IK_Solutions[0] == -1) {
            std::cout << "No solution found." << std::endl;
            solution_found = false;
        } else {
            std::cout << "IK solutions: ";
            std::cout << fmod(IK_Solutions[0],360) + 135 << ", " << fmod(IK_Solutions[1],360) + 135 << ", " << fmod(IK_Solutions[2],360) + 90 << ", " << fmod(IK_Solutions[3],360) + 90 << ", " << fmod(IK_Solutions[4],360) + 90 << std::endl;
            solution_found = true;
        }


        double smoothness = 0.3;
        if(false and solution_found){
            pwm.setSmoothServoAngle(BASE, MS62_SERVO, IK_Solutions[0] + 135, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(SHOULDER, MS62_SERVO_A, IK_Solutions[1] + 45.0, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(UPPER_ARM, DM996_SERVO, IK_Solutions[2] + 90, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(FOREARM, DM996_SERVO, IK_Solutions[3] + 90, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(WIRST, DM996_SERVO, IK_Solutions[4] + 90, smoothness);
            usleep(20);
            
            //pwm.setSmoothServoAngle(FINGER, DM996_SERVO, rt, 2);

        } else if(true) {
            pwm.setSmoothServoAngle(BASE, MS62_SERVO, 135, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(SHOULDER, MS62_SERVO_A, 141, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(UPPER_ARM, DM996_SERVO, 60, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(FOREARM, DM996_SERVO, 90, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(WIRST, DM996_SERVO, 90, smoothness);
            usleep(20);
            //pwm.setSmoothServoAngle(FINGER, DM996_SERVO, RS, 2);
        
        } else {
            pwm.setSmoothServoAngle(BASE, MS62_SERVO, 0, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(SHOULDER, MS62_SERVO_A, 0, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(UPPER_ARM, DM996_SERVO, 0, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(FOREARM, DM996_SERVO, 0, smoothness);
            usleep(20);
            pwm.setSmoothServoAngle(WIRST, DM996_SERVO, 0, smoothness);
            usleep(20);
        }


        usleep(100000);
    }

    for(int i = 0; i < 50; i++) {
        
        pwm.setSmoothServoAngle(BASE, MS62_SERVO, 135, 2);
        usleep(20);
        pwm.setSmoothServoAngle(SHOULDER, MS62_SERVO_A, 141, 2);
        usleep(20);
        pwm.setSmoothServoAngle(UPPER_ARM, DM996_SERVO, 60, 2);
        usleep(20);
        pwm.setSmoothServoAngle(FOREARM, DM996_SERVO, 90, 2);
        usleep(20);
        pwm.setSmoothServoAngle(WIRST, DM996_SERVO, 90, 2);
        usleep(20);
        pwm.setSmoothServoAngle(FINGER, DM996_SERVO, 90, 2);
        usleep(100000);
    }

    // Program stopping
    for (int i = 0; i < 16; i++) {
        pwm.setPWM(i, 0, 4096);   // Turn off channel (full-off via 4096)
    }

    pwm.sleep();                 // Put PCA9685 to sleep to stop all outputs
    std::cout << "Goodbye." << std::endl;

    return 0;

}