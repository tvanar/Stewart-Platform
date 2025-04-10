#include "kinematics.hpp"
#include <BasicLinearAlgebra.h>
using namespace BLA;

void setup() {

    Serial.begin(9600);
    const double BASE_RADIUS = 100.0;
    const double PLATFORM_RADIUS = 80.0;
    const double PHI_BASE = 0.87266;
    const double PHI_PLATFORM = 1.396263;
    double POSE[6] = {0.0,0.0,1.0,0.08726,0.0,0.0};
    while(!Serial);
    Kinematics kinematics(BASE_RADIUS, PLATFORM_RADIUS, PHI_BASE, PHI_PLATFORM);
    BLA::Matrix<6,1,double> m = kinematics.runInverseKinematics(POSE);

    double legs[6] = {31.17,31.93,31.03,30.88,31.50,30.88};
    double guess[6] = {0.0,0.0,1.0,0.0826,0.0,0.0};
    
    kinematics.forwardKinematics(guess,legs);
    for(int i = 0;i < 6;i++) {
      Serial.print(kinematics.position[i],5);
      Serial.print(" ");
    }

    Serial.println(kinematics.iter);
    // BLA::Matrix<3,3,double> t = kinematics.getR();
    // print3M(t);

    
    }
  
    
  void printM(BLA::Matrix<6,6,double> m) {
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 6; j++) {
        Serial.print(m(i,j), 10); // Print with specified precision
        if(j < 6 - 1) Serial.print(" "); // Add space between elements
      }
      Serial.println(); // New line after each row
    }
  }

  void print3M(BLA::Matrix<3,3,double> m) {
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++) {
        Serial.print(m(i,j), 10); // Print with specified precision
        if(j < 3 - 1) Serial.print(" "); // Add space between elements
      }
      Serial.println(); // New line after each row
    }
  }
    
    void loop() {
    
    
    }
