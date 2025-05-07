/*
#ifndef Output2Keystrokes_HPP
#define Output2Keystrokes_HPP

#include <Arduino.h>
#include <math.h>
//#include <BasicLinearAlgebra.h>
//#include "kinematics.hpp"
#include <Keyboard.h>

//Set up our variables of the position and the angles.
double x_val, y_val, z_val, roll, yaw, pitch; //deklarerar värdena på själva positionsvärdena och vinklarna.
const double x_0, y_0, z_0, roll_0, yaw_0, pitch_0; //Sätter gränserna för när ett knapptryck på tangentbordet ska ske. _0 = nollzon.

void setupFunction() {
  Keyboard.begin(KeyboardLayout_sv_SE);//startar keyboard 
  x_0 = 500; //de här värdena ska jag ta från en vektor som kommer från Enars kod.
  y_0 = 500;
  z_0 = 500;
  roll_0 = 500;
  yaw_0 = 500;
  pitch_0 = 500;

}

void loopFunction() { // Here i will make a simple example to try out the concept, wether we go beyond the negative limit or positive limit in any direction doesn't matter in this simple example, they both give the same result. Can make more complex later if we want to.
  if(fabs(x_val) < x_0 && fabs(y_val) < y_0 && fabs(z_val) < z_0){
    //do nothing
  } 
  else{
    if(fabs(x_val) >  x_0){ //if we are far enough in the positive or negative direction, press key.
      Keyboard.press(a);//press a tangent
      delay(1000);
      Keyboard.release();
  }
    if(fabs(y_val) > y_0){ //if we are far enough in the positive or negative direction, press key.
      Keyboard.press(b);//press a different tangent
      delay(1000);
      Keyboard.release();
  }
    if(fabs(z_val) > z_0){ //if we are far enough in the positive or negative direction, press key.
      Keyboard.press(c);//press a different tangent
      delay(1000);
      Keyboard.release();
  }
    if(fabs(roll_val) > roll_0){ //if we are far enough in the positive or negative direction, press key.
      Keyboard.press(c);//press a different tangent
      delay(1000);
      Keyboard.release();
  }
    if(fabs(yaw_val) > yaw_0){ //if we are far enough in the positive or negative direction, press key.
      Keyboard.press(d);//press a different tangent
      delay(1000);
      Keyboard.release();
  } 
    if(fabs(pitch_val) > pitch_0){ //if we are far enough in the positive or negative direction, press key.
      Keyboard.press(e);//press a different tangent
      delay(1000);
      Keyboard.release();
  }

  }
  

}


//Här skriver vi kod för att ta alla positionsvärden och vinkelvärden från Enars vektor och....
//....beroende på dessa värden så ska vi ge olika tangentslag. Typ om musen är väldigt åt höger..
//.. så ska kanske tangent d tryckas på, och sen är nästa steg pwm som hade gjort att beroende...
//..på hur mycket åt höger vi går, desto mer frekvent ska tangenten tryckas.

#endif
*/