#include "Controller.h"
#include "../Utilities/Utilities.h"

#include <cstdint>
#include <string>
#include <SDL2/SDL.h>
#include <iostream>
#include <unistd.h>
#include <cmath>
#include <limits.h>

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
    #define MAX_RETRIES 30

    int16_t X_LS_VALUE = 0;
    int16_t Y_LS_VALUE = 0;
    int16_t LT_VALUE   = 0;
    int16_t X_RS_VALUE = 0;
    int16_t Y_RS_VALUE = 0;
    int16_t RT_VALUE   = 0;
}

Controller::Controller() = default;

Controller::~Controller() = default;



bool Controller::initialize_SDL() {
    if (SDL_Init(SDL_INIT_JOYSTICK) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << std::endl;
        return false;
    }

    SDL_JoystickEventState(SDL_ENABLE);
    return true;
}

bool Controller::checkController() {
    uint retries = 0;
    std::cout << "Looking for game-controller" << std::flush;

    while (SDL_NumJoysticks() < 1) {
        std::cout << "." << std::flush;
        retries++;
        if (retries > MAX_RETRIES) { // Will wait 1 minute
            std::cout << std::endl;
            std::cerr << "No game-controller found" << std::endl;
            return false;
        }
        sleep(2);
    }

    std::cout << std::endl;
    std::cout << "Found!" << std::endl;
    return true;
}

SDL_Joystick* Controller::getGameController() {
    return SDL_JoystickOpen(0);
}

void Controller::handleJoyButtons(SDL_Event e) {
    switch ((int)e.jbutton.button) 
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
            break;
        case PLU:
            break;
    }
}

void Controller::handleJaxis(SDL_Event e) {
    switch ((int)e.jaxis.axis) 
    {
        case X_LS:
            X_LS_VALUE = e.jaxis.value;
            break;
        case Y_LS:
            Y_LS_VALUE = e.jaxis.value;
            break;
        case LT:
            LT_VALUE = e.jaxis.value;
            break;
        case X_RS:
            X_RS_VALUE = e.jaxis.value;
            break;
        case Y_RS:
            Y_RS_VALUE = e.jaxis.value;
            break;
        case RT:
            RT_VALUE = e.jaxis.value;
            break;
    }
}



float Controller::calculateJoyAngle(int16_t joyX, int16_t joyY) {
    return fmod(atan2(joyY, joyX) * 180/M_PI + 360, 360);
}

float Controller::getLSAngle() {
    return calculateJoyAngle(X_LS_VALUE, Y_LS_VALUE);
}

float Controller::getRSAngle() {
    return calculateJoyAngle(X_RS_VALUE, Y_RS_VALUE);
}