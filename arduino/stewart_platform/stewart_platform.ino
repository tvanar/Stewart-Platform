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

    BLA::Matrix<3,1> base_attachments[6];
    BLA::Matrix<3,1> platform_attachments[6];
    BLA::Matrix<3,1> legs[6];
    float legs_length[6];
    BLA::Matrix<3,3> r;


  /* 
  * Sets up the initial coordinates of the system
  */
    void setUpPlatform() {
      for(int i = 0; i < NUM_LEGS; i++) {
        int j = (i % 2 == 1) ? -1 : 1;
        Kinematics::base_attachments[i] = {base_radius * cos(angles_rad[i] + j * phi_base / 2.0f),base_radius * sin(angles_rad[i] + j * phi_base / 2.0f), 0.0f};
        Kinematics::platform_attachments[i] = {platform_radius * cos(angles_rad[i] + j * phi_platform / 2.0f),platform_radius * sin(angles_rad[i] + j * phi_platform / 2.0f), 0.0f};
      }
    }

    /*
    * Calculates the L2 norm of vec and inputs the result in leg_len
    */
    void norm(BLA::Matrix<3,1> leg_vec[NUM_LEGS], float leg_len[NUM_LEGS]) {
      for(int i = 0; i < NUM_LEGS; i++) {
        leg_len[i] = sqrt(leg_vec[i](0)*leg_vec[i](0) + leg_vec[i](1)*leg_vec[i](1) + leg_vec[i](2)*leg_vec[i](2)); 
      }
    }

    /*
    * Calculate Rotation matrix and save in r
    */
    void Rotation_as_euler(float angles[DIM]) {
      float cz = cos(angles[0]), sz = sin(angles[0]);  // Z-axis rotation
      float cy = cos(angles[1]), sy = sin(angles[1]);  // Y-axis rotation
      float cx = cos(angles[2]), sx = sin(angles[2]);  // X-axis rotation
      Kinematics::r = {cy*cz,cz*sx*sy - cx*sz,sx*sz + cx*cz*sy,
                cy*sz,cx*cz + sx*sy*sz,cx*sy*sz - cz*sx,
                -sy,cy*sx,cx*cy};
    }

    /*
    * Calculates leg lengths based on a pose
    */
    void InverseKinematics(float pose[NUM_LEGS]){
      float euler_angles[3] = {pose[5],pose[4],pose[3]};
      Kinematics::Rotation_as_euler(euler_angles);
      BLA::Matrix<3,1> pose_as_matrix = {pose[0],pose[1],pose[2]};
      for(int i = 0;i < NUM_LEGS ;i++){
        int j = (i % 2 == 1) ? -1 : 1;
        Kinematics::base_attachments[i] = {base_radius * cos(angles_rad[i] + j * phi_base / 2.0f),base_radius * sin(angles_rad[i] + j * phi_base / 2.0f), 0.0f};
        Kinematics::platform_attachments[i] = {platform_radius * cos(angles_rad[i] + j * phi_platform / 2.0f),platform_radius * sin(angles_rad[i] + j * phi_platform / 2.0f), 0.0f};
        Kinematics::legs[i] = pose_as_matrix + (r * Kinematics::platform_attachments[i]) - Kinematics::base_attachments[i]; // Leg vectors
        Kinematics::platform_attachments[i] = Kinematics::legs[i] + Kinematics::base_attachments[i]; // Moving platform with respect to base
      }
    }

  public:
    // Constructor
    Kinematics(float base_radius, float platform_radius, float phi_base, float phi_platform) : base_radius(base_radius), platform_radius(platform_radius), phi_base(phi_base), phi_platform(phi_platform) {
      // setUpPlatform();
    }

    BLA::Matrix<6,1> runInverseKinematics(float pose[NUM_LEGS]) {
      Kinematics::InverseKinematics(pose);
      Kinematics::norm(Kinematics::legs,Kinematics::legs_length);
      BLA::Matrix<6,1> m = {Kinematics::legs_length[0],Kinematics::legs_length[1],Kinematics::legs_length[2],Kinematics::legs_length[3],Kinematics::legs_length[4],Kinematics::legs_length[5]};
      return m;
    }

    
};

void setup() {

Serial.begin(9600);
const float BASE_RADIUS = 100.0f;
const float PLATFORM_RADIUS = 80.0f;
const float PHI_BASE = 0.87266f;
const float PHI_PLATFORM = 1.396263f;
const float POSE[6] = {0.0f,0.0f,1.0f,0.08726f,0.0f,0.0f};

  // Instantiate the Kinematics object locally within setup()
Kinematics kinematics(BASE_RADIUS, PLATFORM_RADIUS, PHI_BASE, PHI_PLATFORM);

BLA::Matrix<6,1> m = kinematics.runInverseKinematics(POSE);
while(!Serial);
Serial.println(m);

}


void loop() {


}
