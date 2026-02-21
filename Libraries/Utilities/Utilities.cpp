#include "Utilities.h"

#include <iostream>

float constrain(double x, double a, double b) {
    if(x < a) {
        return a;
    } else if(x > b) {
        return b;
    } else {
        return x;
    }
}

float map(double x, double fromLow, double fromHigh, double toLow, double toHigh) {
  return toLow + (x-fromLow)*(toHigh-toLow)/(fromHigh-fromLow);
}
