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
    float calculateJoyVector(int16_t joyX, int16_t joyY);
    int16_t getLSX();
    int16_t getLSY();
    int16_t getRSX();
    int16_t getRSY();
    int16_t getLT();
    int16_t getRT();
    bool getProgramState();
    float getLSAngle();
    float getRSAngle();
    float getLSVector();
    float getRSVector();
    float getLTCurve();
    float getRTCurve();
    float trigger_curves(float value);

private:
    SDL_Joystick* joystick_ = nullptr;
};