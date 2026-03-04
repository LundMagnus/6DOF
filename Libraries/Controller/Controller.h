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
    bool openJoystick(int index = 0);
    SDL_Joystick* getGameController();
    void setJoystick(SDL_Joystick* joy) { joystick_ = joy; }
    void handleJoyButtons(SDL_Event e);
    void handleJaxis(SDL_Event e);
    void updateAxes();

    float calculateJoyAngle(int16_t joyX, int16_t joyY);
    int16_t getLSX();
    int16_t getLSY();
    float getLSAngle();
    float getRSAngle();

private:
    SDL_Joystick* joystick_ = nullptr;
};