#include "kinematics.hpp"

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
    
    Serial.println(kinematics.getJacobian());
    
    Serial.print("Unit leg "); Serial.print(i); Serial.print(": ");
    Serial.print(unit_legs[i](0)); Serial.print(", ");
    Serial.print(unit_legs[i](1)); Serial.print(", ");
    Serial.print(unit_legs[i](2)); Serial.println();
    
    Serial.print("Platform attachment "); Serial.print(i); Serial.print(": ");
    Serial.print(platform_attachments[i](0)); Serial.print(", ");
    Serial.print(platform_attachments[i](1)); Serial.print(", ");
    Serial.print(platform_attachments[i](2)); Serial.println();
    
    }
    
    
    
    
    void loop() {
    
    
    }