#ifndef Output2Keystrokes_HPP
#define Output2Keystrokes_HPP

#include <Arduino.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#include "kinematics.hpp"
#include <Keyboard.h>

// Position och rotation (från vincent och einar)
double x_val, y_val, z_val, roll, yaw, pitch;

// threshhold (måste ändras till rimligt vinklar 0->360 i grader)
double x_0 = 500;
double y_0 = 500;
double z_0 = 500;
double roll_0 = 0;
double yaw_0 = 0;
double pitch_0 = 0;

const int MAX_DELAY = 1000; //1 sekund
const int MIN_DELAY = 100;

void setupFunction() {
  Keyboard.begin(KeyboardLayout_sv_SE);
}

void loopFunction() {
  //ändra x_val och de andra
  // mappning av de olika valen
  checkAndPress(x_val, x_0, 'd', 'a');     // X+ = höger, X- = vänster
  checkAndPress(y_val, y_0, 'w', 's');     // Y+ = fram, Y- = bak
  checkAndPress(z_val, z_0, 'o', 'l');     // Z+ = upp, Z- = ner
  checkAndPress(roll, roll_0, 'q', 'e');   // Roll+ = medsold, Roll- = motsols
  checkAndPress(yaw, yaw_0, 'u', 'j');     // Yaw+ = höger, Yaw- = vänster
  checkAndPress(pitch, pitch_0, 'i', 'k'); // Pitch+ = upp, Pitch- = ner
}

#endif
