#include <math.h>
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

const int NUMLEGS = 6;
const float VREF = 5.0f;
const int analog_pins[6] = {A0, A1, A2, A3, A4, A5};
const float ADC_TO_VOLT = VREF / 1023.0f;
float voltages[NUMLEGS];
int raw_voltage;
int pin;
float position[6]; 

void setup() {
    Serial.begin(115200);
    analogReference(DEFAULT);
    for(int i = 0; i<NUMLEGS ;i++) {
      pinMode(analog_pins[i],INPUT);
    }
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
  
}


  
