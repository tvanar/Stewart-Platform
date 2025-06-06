#include <math.h>
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "JacobOutput2Keystrokes_HPP.hpp"
using namespace BLA;

const int NUMLEGS = 6;
const float VREF = 5.0f;
const int analog_pins[6] = {A0, A1, A2, A3, A4, A5};
const float ADC_TO_VOLT = VREF / 1023.0f;
float voltages[NUMLEGS];
int raw_voltage;
int pin;
float position[6]; 
// Position och rotation (från vincent och einar)
double x_val, y_val, z_val, roll, yaw, pitch;

// threshhold (måste ändras till rimligt vinklar 0->360 i grader)
double x_0 = 14;
double y_0 = 20;
double z_0 = 4;
double roll_0 = 3;
double yaw_0 = 5;
double pitch_0 = 7;
double vector[6];


void setup() {
    Keyboard.begin(KeyboardLayout_sv_SE);
    Serial.begin(115200);
    analogReference(DEFAULT);
    for(int i = 0; i<NUMLEGS ;i++) {
      pinMode(analog_pins[i],INPUT);
    }
    pinMode(leftClickPin, INPUT_PULLUP);
    pinMode(rightClickPin, INPUT_PULLUP);
    Mouse.begin();
    while(!Serial);
    delay(10000);
  }


void loop() {

  // READ VOLTAGES
  for(int i = 0; i < NUMLEGS; i++) {
    pin = analog_pins[i];
    analogRead(pin);
    delayMicroseconds(10);
    raw_voltage = analogRead(pin);
    voltages[i] = raw_voltage * ADC_TO_VOLT;
  }
  // for(int i = 0; i < 6; i++) {
  //  Serial.print(voltages[i],5);
  //  Serial.print(" ");
  // }
  // Serial.println("");
  // delay(100);

  // SEND VOLTAGES TO PC
  Serial.write((uint8_t*)voltages, sizeof(voltages));
  
  delay(100);
  // RECEIVE POSITION
  if (Serial.available() >= 24) {  // Check if we have 24 bytes (6 floats)
    byte receivedBytes[24];
    Serial.readBytes(receivedBytes, 24);
    memcpy(position, receivedBytes, sizeof(position));
  }

  x_val = position[0];
  y_val = position[1];
  z_val = position[2]; //ändra x_val och de andra
  roll = position[3];
  pitch = position[4]; // mappning av de olika valen
  yaw = position[5];

  checkAndPress(x_val+16, x_0, 'w', 's');     // X+ = fram, X- = bak godkänd
 checkAndPress(y_val-4, y_0, 'a', 'd');     // Y+ = vänster, Y- = höger
  checkAndPress(z_val-154, z_0, 'o', 'l');     // Z+ = upp, Z- = ner
checkAndPress(roll, roll_0, 'q', 'e');   // Roll+ = medsold, Roll- = motsols
 checkAndPress(pitch-7, yaw_0, 'u', 'j');     // Yaw+ = höger, Yaw- = vänster
 //checkAndPress(yaw+2, pitch_0, 'i', 'k'); // Pitch+ = upp, Pitch- = ner
  mouseClick(); // y = vänsterklick h = högerklick
}


  
