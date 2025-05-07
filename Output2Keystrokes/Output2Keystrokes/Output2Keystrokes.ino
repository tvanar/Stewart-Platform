
//#include "Output2Keystrokes.hpp" //Här inkluderar vi c++ filen med alla våra funktioner och kod i.
//#include <Keyboard.h>
//Jag gör såhär att jag skriver all koden utan funktioner här först, gör sen om det till funktioner som kan kodas i hpp filen och bara använder de funktionerna här sen.

//Set up our variables of the position and the angles.
//double x_val, y_val, z_val, roll, yaw, pitch; //deklarerar värdena på själva positionsvärdena och vinklarna.
//const double x_0, y_0, z_0, roll_0, yaw_0, pitch_0; //Sätter gränserna för när ett knapptryck på tangentbordet ska ske. _0 = nollzon.

void setup() {
  Serial.begin(9600);
/*
 Output2Keystrokes.setupFunction();
*/

}

void loop() { // Here i will make a simple example to try out the concept, wether we go beyond the negative limit or positive limit in any direction doesn't matter in this simple example, they both give the same result. Can make more complex later if we want to.
 //Output2Keystrokes.loopFunction();
  Serial.print("Hej");
}
