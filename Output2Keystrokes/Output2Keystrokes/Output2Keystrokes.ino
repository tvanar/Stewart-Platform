#include "Output2Keystrokes.hpp" //Här inkluderar vi c++ filen med alla våra funktioner och kod i.
#include <keyboard.h>
//Jag gör såhär att jag skriver all koden utan funktioner här först, gör sen om det till funktioner som kan kodas i hpp filen och bara använder de funktionerna här sen.

//Set up our variables of the position and the angles.
double x_val, y_val, z_val, roll, yaw, pitch; //deklarerar värdena på själva positionsvärdena och vinklarna.
const double x_0, y_0, z_0, roll_0, yaw_0, pitch_0; //Sätter gränserna för när ett knapptryck på tangentbordet ska ske. _0 = nollzon, _neg = negativt värde, _pos = positivt värde.

void setup() {
  Keyboard.begin(KeyboardLayout_sv_SE);//startar keyboard 
  x_0 = 500;
  y_0 = 500;
  z_0 = 500;
  roll_0 = 500;
  yaw_0 = 500;
  pitch_0 = 500;

}

void loop() { // Here i will make a simple example to try out the concept, wether we go beyond the negative limit or positive limit in any direction doesn't matter in this simple example, they both give the same result. Can make more complex later if we want to.
  if(fabs(x_val) < x_0 && fabs(y_val) < y_0 && fabs(z_val) < z_0){
    Serial.println("Musen är i nolläge");
  }
  if(fabs(x_val) >  x_0){ //if we are far enough in the positive or negative direction, press key.
    keyboard.press(a);//press a tangent
    delay(100);
    keyboard.release();
  }
  if(fabs(y_val) > y_0){ //if we are far enough in the positive or negative direction, press key.
    keyboard.press(b);//press a different tangent
    delay(100);
    keyboard.release();
  }
  if(fabs(z_val) > z_0){ //if we are far enough in the positive or negative direction, press key.
    keyboard.press(c);//press a different tangent
    delay(100);
    keyboard.release();
  }
  if(fabs(roll_val) > roll_0){ //if we are far enough in the positive or negative direction, press key.
    keyboard.press(c);//press a different tangent
    delay(100);
    keyboard.release();
  }
  if(fabs(yaw_val) > yaw_0){ //if we are far enough in the positive or negative direction, press key.
    keyboard.press(d);//press a different tangent
    delay(100);
    keyboard.release();
  } 
  if(fabs(pitch_val) > pitch_0){ //if we are far enough in the positive or negative direction, press key.
    keyboard.press(e);//press a different tangent
    delay(100);
    keyboard.release();
  }

}
