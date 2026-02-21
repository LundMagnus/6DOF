#pragma once

#include <cstdint>
#include <string>
#include <SDL2/SDL.h>

class Controller {
public:
    explicit Controller();
    ~Controller();

    bool initialize_SDL();
    bool checkController();
    SDL_Joystick* getGameController();
    void handleJoyButtons(SDL_Event e);
    void handleJaxis(SDL_Event e);

    float calculateJoyAngle(int16_t joyX, int16_t joyY);
    float getLSAngle();
    float getRSAngle();
};