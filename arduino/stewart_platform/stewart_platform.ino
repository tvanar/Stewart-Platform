#include "kinematics_alt.hpp"
#include <BasicLinearAlgebra.h>
using namespace BLA;

void setup() {

    Serial.begin(9600);
    const float BASE_RADIUS = 100.0f;
    const float PLATFORM_RADIUS = 80.0f;
    const float PHI_BASE = 0.87266f;
    const float PHI_PLATFORM = 1.396263f;
    const float POSE[6] = {0.0f,0.0f,1.0f,0.08726f,0.0f,0.0f};
    while(!Serial);
      // Instantiate the Kinematics object locally within setup()
    Kinematics kinematics(BASE_RADIUS, PLATFORM_RADIUS, PHI_BASE, PHI_PLATFORM);
    BLA::Matrix<6,1> m = kinematics.runInverseKinematics(POSE);
    
    
    Serial.println(m);

    Serial.println(kinematics.getR());
    
    Serial.println(kinematics.getJacobian());

    BLA::Matrix<3,6> m2 = kinematics.getUnitLegs();

    Serial.println(m2);

    Serial.println(kinematics.getLegs());

    
    }
  
    
    
    
    void loop() {
    
    
    }
