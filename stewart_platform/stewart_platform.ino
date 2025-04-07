#include "kinematics_double.hpp"
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
      // Instantiate the Kinematics object locally within setup()
    Kinematics kinematics(BASE_RADIUS, PLATFORM_RADIUS, PHI_BASE, PHI_PLATFORM);
    BLA::Matrix<6,1,double> m = kinematics.runInverseKinematics(POSE);

    
    Serial.println(m);

    double POSE2[6] = {0.0,0.1,1.0,0.0,0.0,0.0};

    BLA::Matrix<6,1,double> m2 = kinematics.runInverseKinematics(POSE2);

    Serial.println(m2);

    // BLA::Matrix<3,3,double> t =kinematics.getR();

    // Serial.println(t(1,1),10);
    
    // Matrix<6,6,double> j = kinematics.getJacobian();
    // printM(j);

    // BLA::Matrix<3,6,double> m2 = kinematics.getUnitLegs();
    // Serial.println("");
    // Serial.println(m2);

    // Serial.println(kinematics.getLegs());

    
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
    
    void loop() {
    
    
    }
