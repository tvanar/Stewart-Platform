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

const int MAX_DELAY = 1000; //1 sekund
const int MIN_DELAY = 100;

bool LRS = HIGH;
bool LLS = HIGH;
const int leftClickPin = 7;
const int rightClickPin = 8;

int computeDelay(double value, double threshold) {
  double delta = fabs(value) - threshold;
  if (delta <= 0) return 0;
  double delayTime = MAX_DELAY - (delta / threshold) * (MAX_DELAY - MIN_DELAY);
  return constrain(delayTime, MIN_DELAY, MAX_DELAY);
}

void checkAndPress(double value, double threshold, char posKey, char negKey) {
  if (fabs(value) > threshold) {
     delay(10);
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

void mouseClick(){
  bool LS = digitalRead(leftClickPin);
  bool RS = digitalRead(rightClickPin);

  if (LS == LOW && LLS == HIGH) {
    Keyboard.press('y'); 
  }
  if (LS == HIGH && LLS == LOW) {
    Keyboard.release('y');
  }
  if (RS == LOW && LRS == HIGH) {
    Keyboard.press('h');  // 
  }
  if (RS == HIGH && LRS == LOW) {
    Keyboard.release('h');
  }
  LLS = LS;
  LRS = RS;
}

#endif