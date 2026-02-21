#pragma once

#include <cstdint>
#include <string>
#include <SDL2/SDL.h>

class Controller {
public:
    explicit Controller();
    ~Controller();

    void initialize_SDL();
    void checkController();
    SDL_Joystick getGameController();
    void handleJoyButtons(SDL_Event e);
    void handleJaxis(SDL_Event e);
};