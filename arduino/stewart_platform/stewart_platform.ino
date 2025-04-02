#include <BasicLinearAlgebra.h>
using namespace BLA;

const int NUM_LEGS = 6;
const int DIM = 3;
const float angles_rad[6] = {0.0f, 2.0943951f, 2.0943951f, 4.1887902f, 4.1887902f, 0.0f};

class Kinematics {
  private:
    float base_radius;
    float platform_radius;
    float phi_base; // RADIANS
    float phi_platform; // RADIANS

    BLA::Matrix<1,3> base_attachments[6];
    BLA::Matrix<1,3> platform_attachments[6];
    BLA::Matrix<1,3> legs[6];
    float legs_lengths[6];
    BLA::Matrix<3,3> r;


  /* 
  * Sets up the initial coordinates of the system
  */
    void setUpPlatform() {
      for(int i = 0; i < NUM_LEGS; i++) {
        int j = (i % 2 == 1) ? -1 : 1;
        base_attachments[i] = {base_radius * cos(angles_rad[i] + j * phi_base / 2.0f),base_radius * sin(angles_rad[i] + j * phi_base / 2.0f), 0}
        platform_attachments[i] = {platform_radius * cos(angles_rad[i] + j * phi_platform / 2.0f),platform_radius * sin(angles_rad[i] + j * phi_platform / 2.0f), 0}
      }
    }

    /*
    * Calculate Rotation matrix and save in r
    */
    void Rotation_as_euler(float angles[3], BLA::Matrix result) {
      float cz = cos(angles[0]), sz = sin(angles[0]);  // Z-axis rotation
      float cy = cos(angles[1]), sy = sin(angles[1]);  // Y-axis rotation
      float cx = cos(angles[2]), sx = sin(angles[2]);  // X-axis rotation
      result = {cy*cz,cz*sx*sy - cx*sz,sx*sz + cx*cz*sy,
                cy*sz,cx*cz + sx*sy*sz,cx*sy*sz - cz*sx,
                -sy,cy*sx,cx*cy};
    }

    /*
    * Calculates leg lengths based on a pose
    */
    void InverseKinematics(float pose[6],float lengths[6]){
      Rotation_as_euler({pose[3], pose[4], pose[5]},r)
      for(int i = 0;i < NUM_LEGS ;i++){

        legs[i] = 
      }
    }

  public:
    // Constructor
    Kinematics(float base_radius, float platform_radius, float phi_base, float phi_platform) : base_radius(base_radius), platform_radius(platform_radius), phi_base(phi_base), phi_platform(phi_platform) {
      setUpPlatform()
    }

    
};


void setup(){

}


void loop(){

}