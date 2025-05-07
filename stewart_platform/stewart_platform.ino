#include <math.h>
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

const int NUMLEGS PROGMEM = 6;
const float VREF PROGMEM = 5.0f;
const int analog_pins[6] PROGMEM = {A0, A1, A2, A3, A4, A5};
const float ADC_TO_VOLT = VREF / 1023.0f;
float voltages[NUMLEGS];
int raw_voltage;
float position[6]; 

void setup() {
    Serial.begin(9600);
    

    while(!Serial);
  }


void loop() {

  // READ VOLTAGES
  for(int i = 0; i < NUMLEGS; i++) {
    raw_voltage = analogRead(analog_pins[i]);
    voltages[i] = raw_voltage * ADC_TO_VOLT;
  }
  for(int i = 0; i < 6; i++) {
   Serial.print(voltages[i],5);
   Serial.print(" ");
  }
  Serial.println("");
  delay(500);
  // // SEND VOLTAGES TO PC
  // Serial.write((uint8_t*)voltages, sizeof(voltages));
  
  // delay(100);
  // // RECEIVE POSITION
  // if (Serial.available() >= 24) {  // Check if we have 24 bytes (6 floats)
  //   byte receivedBytes[24];
  //   Serial.readBytes(receivedBytes, 24);
  //   memcpy(position, receivedBytes, sizeof(position));
  // }

}
  
