#include <ArduinoEigen.h>


class Kinematics {
  private:
    float base_radius;
    float platform_radius;
    float phi_base; // RADIANS
    float phi_platform; // RADIANS

    int InverseKinematics() {
      return 3;
    }
    





  public:
    // Constructor
    Kinematics(float base_radius, float platform_radius, float phi_base, float phi_platform) {
      this->base_radius = base_radius;
      this->platform_radius = platform_radius;
      this->phi_base = phi_base;
      this->phi_platform = phi_platform;
    }



};















void setup(){

}


void loop(){

}