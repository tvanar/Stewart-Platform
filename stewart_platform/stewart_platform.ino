#include "kinematics.hpp"
#include <BasicLinearAlgebra.h>
using namespace BLA;


template<int rows, int cols>
void printM(const BLA::Matrix<rows, cols, double>& matrix);


void setup() {
    Serial.begin(9600);
    const double BASE_RADIUS = 10.0;
    const double PLATFORM_RADIUS = 8.0;
    const double PHI_BASE = 0.87266;
    const double PHI_PLATFORM = 1.396263;
    double POSE[6] = {0.0,0.0,1.0,0.08726,0.0,0.0};
    while(!Serial);
    Kinematics kinematics(BASE_RADIUS, PLATFORM_RADIUS, PHI_BASE, PHI_PLATFORM);
    BLA::Matrix<6,1,double> m = kinematics.runInverseKinematics(POSE);
    kinematics.calculateJacobian();
    double legs[6] = {3.39317,3.52455,3.32389,3.17716,3.11060,3.11783};
    double guess[6] = {0.0,0.0,1.0,0.001,0.0,0.0};
    
    // // printM(m);
    kinematics.forwardKinematics(guess,legs);
    
    for(int i = 0;i < 6;i++) {
      Serial.print(kinematics.position[i],5);
      Serial.print(" ");
    }

    
  }

void loop() {
  
}
  
    
template<int rows, int cols>
void printM(const BLA::Matrix<rows, cols, double>& matrix) {  
    Serial.print("[");
    for (int i = 0; i < rows; ++i) {
        if (i > 0) Serial.print(" ");
        Serial.print("[");
        for (int j = 0; j < cols; ++j) {
            Serial.print(matrix(i, j), 5);
            if (j < cols - 1) {
                Serial.print(", ");
            }
        }
        Serial.print("]");
        if (i < rows - 1) {
            Serial.println(",");
        }
    }
    Serial.println("]");
    Serial.println();
}
