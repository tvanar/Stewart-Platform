/*
#ifndef Output2Keystrokes_HPP
#define Output2Keystrokes_HPP
*/

#ifndef JacobOutput2Keystrokes_HPP
#define JacobOutput2Keystrokes_HPP

#include <Arduino.h>
#include <math.h>
//#include <BasicLinearAlgebra.h>
//#include "kinematics.hpp"
#include <Keyboard.h>
#include <Mouse.h>

const int MAX_DELAY = 1000; //1 sekund
const int MIN_DELAY = 100;

const int leftClickPin = 8;
const int rightClickPin = 7;

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

void mouseClick() {
  if (digitalRead(leftClickPin) == HIGH) {
    Mouse.press(MOUSE_LEFT);
    delay(100);
    Mouse.release(MOUSE_LEFT);
  }

  if (digitalRead(rightClickPin) == HIGH) {
    Mouse.press(MOUSE_RIGHT);
    delay(150);
    Mouse.release(MOUSE_RIGHT);
  }

  //delay(10);
}


#endif