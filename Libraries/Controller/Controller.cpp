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
    #define X_LR 3
    #define Y_LR 4
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
    int16_t X_LR_VALUE = 0;
    int16_t Y_LR_VALUE = 0;
    int16_t RT_VALUE   = 0;
}



void initialize_SDL() {
    SDL_Init(SDL_INIT_JOYSTICK);
}

void checkController() {
    uint retries = 0;
    std::cout << "Looking for game-controller";

    while (SDL_NumJoysticks() < 1) {
        std::cout << ".";
        retries++;
        if(retries > MAX_RETRIES) { // Will wait 1 minute
            abort;
        }
        sleep(2);
    }

    std::cout << std::endl;
    std::cout << "Found!" << std::endl;
    return;
}

SDL_Joystick* getGameController() {
    return SDL_JoystickOpen(0);
}

void handleJoyButtons(SDL_Event e) {
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

void handleJaxis(SDL_Event e) {
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
        case X_LR:
            X_LR_VALUE = e.jaxis.value;
            break;
        case Y_LR:
            Y_LR_VALUE = e.jaxis.value;
            break;
        case RT:
            RT_VALUE = e.jaxis.value;
            break;
    }
}

float calculateJoyAngle(int16_t joyX, int16_t joyY) {

    float joyXPer = map(joyX, SHRT_MIN, SHRT_MAX, 0, 100);
    float joyYPer = map(joyY, SHRT_MIN, SHRT_MAX, 0, 100);

    return ;
}
