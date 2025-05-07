//#ifndef Output2Keystrokes_HPP
//#define Output2Keystrokes_HPP
#include "JacobOutput2Keystrokes_HPP.hpp"
//#ifndef JacobOutput2Keystrokes_HPP
//#define JacobOutput2Keystrokes_HPP

#include <Arduino.h>
#include <math.h>
//#include <BasicLinearAlgebra.h>
//#include "kinematics.hpp"
//#include <Keyboard.h>

// Position och rotation (från vincent och einar)
double x_val, y_val, z_val, roll, yaw, pitch;

// threshhold (måste ändras till rimligt vinklar 0->360 i grader)
double x_0 = 50;
double y_0 = 50;
double z_0 = 50;
double roll_0 = 0;
double yaw_0 = 0;
double pitch_0 = 0;
int hej = 0;
double vector[6];

void setup() {
  Serial.begin(9600); 
  Keyboard.begin(KeyboardLayout_sv_SE);
}

void loop() {
    A1[6] = {100, 0, 0, 0, 0, 0}; //Här kan vi uppdatera vektorn som Enar ger oss.
    x_val = A1[0];
    y_val = A1[1];
    z_val = A1[2]; //ändra x_val och de andra
    roll = A1[3];
    yaw = A1[4];
    pitch = A1[5]; // mappning av de olika valen
    checkAndPress(x_val, x_0, 'd', 'a');     // X+ = höger, X- = vänster
    checkAndPress(y_val, y_0, 'w', 's');     // Y+ = fram, Y- = bak
    checkAndPress(z_val, z_0, 'o', 'l');     // Z+ = upp, Z- = ner
    checkAndPress(roll, roll_0, 'q', 'e');   // Roll+ = medsold, Roll- = motsols
    checkAndPress(yaw, yaw_0, 'u', 'j');     // Yaw+ = höger, Yaw- = vänster
    checkAndPress(pitch, pitch_0, 'i', 'k'); // Pitch+ = upp, Pitch- = ner

}