#ifndef Output2Keystrokes_HPP
#define Output2Keystrokes_HPP

#include <Arduino.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#include "kinematics.hpp"
#include <Keyboard.h>

int computeDelay(double value, double threshold) {
  double delta = fabs(value) - threshold;
  if (delta <= 0) return 0;
  double delayTime = MAX_DELAY - (delta / threshold) * (MAX_DELAY - MIN_DELAY);
  return constrain(delayTime, MIN_DELAY, MAX_DELAY);
}

void checkAndPress(double value, double threshold, char posKey, char negKey) {
  if (fabs(value) > threshold) {
    int d = computeDelay(value, threshold);
    if (value > 0) {
      Keyboard.press(posKey);
    } else {
      Keyboard.press(negKey);
    }
    delay(d);
    Keyboard.releaseAll(); 
  }
}