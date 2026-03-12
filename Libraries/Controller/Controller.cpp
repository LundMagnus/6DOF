#include "Controller.h"
#include "../Utilities/Utilities.h"

#include <cstdint>
#include <string>
#include <SDL2/SDL.h>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <limits.h>
#include <chrono>
#include <ctime>

namespace {
    #define X_LS 0
    #define Y_LS 1
    #define LT   2
    #define X_RS 3
    #define Y_RS 4
    #define RT   5
    #define A    0
    #define B    1
    #define Y    2
    #define X    3
    #define LB   4
    #define RB   5
    #define MIN  6
    #define PLU  7
    #define MAX_RETRIES 120

    int16_t X_LS_VALUE = 0;
    int16_t Y_LS_VALUE = 0;
    int16_t LT_VALUE   = 0;
    int16_t X_RS_VALUE = 0;
    int16_t Y_RS_VALUE = 0;
    int16_t RT_VALUE   = 0;
    bool PROGRAMSTATE  = true;
}

Controller::Controller() = default;

Controller::~Controller() {
    if (joystick_) {
        SDL_JoystickClose(joystick_);
        joystick_ = nullptr;
    }
}



bool Controller::initialize_SDL() {
    if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return false;
    }

    SDL_JoystickEventState(SDL_ENABLE);
    return true;
}

bool Controller::checkController() {
    uint retries = 0;
    std::cout << "Looking for game-controller" << std::flush;
    int start_time = std::time(0);

    while (SDL_NumJoysticks() < 1) { // Find controller

        initialize_SDL(); // Keeps re-opening port

        if((start_time - std::time(0)) != 0) {
            std::cout << "." << std::flush;
            retries++;
            start_time = std::time(0);
        }
        
        
        if (retries > MAX_RETRIES) { // Will wait 1 minute
            std::cout << std::endl;
            std::cerr << "No game-controller found after ~60s" << std::endl;
            int num_joysticks = SDL_NumJoysticks();
            std::cerr << "Debug: SDL detected " << num_joysticks << " joystick(s)" << std::endl;
            return false;
        }
       
    }

    std::cout << std::endl;
    
    // Open immediately to prevent controller sleep
    joystick_ = SDL_JoystickOpen(0);
    if (!joystick_) {
        std::cerr << "SDL_JoystickOpen failed: " << SDL_GetError() << std::endl;
        return false;
    }
    
    const char* name = SDL_JoystickName(joystick_);
    std::cout << "Found controller: " << (name ? name : "Unknown") << std::endl;
    return true;
}

bool Controller::openJoystick(int index) {
    if (joystick_) {
        return true;  // Already opened by checkController
    }
    
    joystick_ = SDL_JoystickOpen(index);
    if (!joystick_) {
        std::cerr << "SDL_JoystickOpen failed: " << SDL_GetError() << std::endl;
        return false;
    }
    return true;
}

SDL_Joystick* Controller::getGameController() {
    return joystick_;
}

void Controller::handleJoyButtons(SDL_Event e) {
    int button = -1;
    if (e.type == SDL_JOYBUTTONDOWN) {
        button = static_cast<int>(e.jbutton.button);
    } else if (e.type == SDL_CONTROLLERBUTTONDOWN) {
        button = static_cast<int>(e.cbutton.button);
    } else {
        return;
    }

    switch (button)
    {
        case A:
            break;
        case B:
            break;
        case Y:
            break;
        case X:
            break;
        case LB:
            break;
        case RB:
            break;
        case MIN:
            PROGRAMSTATE = false;
            break;
        case PLU:
            break;
    }
}


void Controller::updateAxes() {
    if (!joystick_) {
        return;
    }

    SDL_JoystickUpdate();
    X_LS_VALUE = SDL_JoystickGetAxis(joystick_, X_LS);
    Y_LS_VALUE = SDL_JoystickGetAxis(joystick_, Y_LS);
    LT_VALUE = SDL_JoystickGetAxis(joystick_, LT);
    X_RS_VALUE = SDL_JoystickGetAxis(joystick_, X_RS);
    Y_RS_VALUE = SDL_JoystickGetAxis(joystick_, Y_RS);
    RT_VALUE = SDL_JoystickGetAxis(joystick_, RT);
}



float Controller::calculateJoyAngle(int16_t joyX, int16_t joyY) {
    return 360 - fmod(atan2(joyY, joyX) * 180/M_PI + 360, 360);
}

int16_t Controller::getLSX() {
    return X_LS_VALUE;
}

int16_t Controller::getLSY() {
    return Y_LS_VALUE;
}

float Controller::getLSAngle() {
    return calculateJoyAngle(X_LS_VALUE, Y_LS_VALUE);
}

float Controller::getRSAngle() {
    return calculateJoyAngle(X_RS_VALUE, Y_RS_VALUE);
}

bool Controller::getProgramState() {
    return PROGRAMSTATE;
}