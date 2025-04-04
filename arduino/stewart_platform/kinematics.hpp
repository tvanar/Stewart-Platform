#ifndef KINEMATICS
#define KINEMATICS

#include <BasicLinearAlgebra.h>
using namespace BLA;

const int NUM_LEGS = 6;
const int DIM = 3;
const float angles_rad[6] = {0.0f, 2.0943951f, 2.0943951f, 4.1887902f, 4.1887902f, 0.0f}; // BASE ANGLES USED FOR PLATFORM

class Kinematics {
  public:
    float base_radius; // RADIUS OF BASE
    float platform_radius; // RADIUS OF PLATFORM
    float phi_base; // RADIANS
    float phi_platform; // RADIANS

    BLA::Matrix<3,1> base_attachments[6]; // COORDINATES OF BASE ATTACHMENTS POINTS
    BLA::Matrix<3,1> platform_attachments[6]; // COORDINATES OF PLATFORM ATTACHMENT POINTS
    BLA::Matrix<3,1> legs[6]; // LEG VECTORS
    BLA::Matrix<3,1> unit_legs[6]; // UNIT LEG VECTORS FOR JACOBIAN
    float legs_length[6]; // L2 NORM OF LEGS
    BLA::Matrix<3,3> r; // ROTATION MATRIX


    // JACOBIAN MATRICES
    BLA::Matrix<6,6> jacobian;
    BLA::Matrix<6,6> jacobian_transpose;
    BLA::Matrix<6,6> j_star;

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
    float norm(BLA::Matrix<3,1> leg_vec) {
      return sqrt(leg_vec(0)*leg_vec(0) + leg_vec(1)*leg_vec(1) + leg_vec(2)*leg_vec(2)); 
    }

    /*
    * Calculate Rotation matrix and stores it in R
    */
    void rotationAsEuler(float angles[DIM]) {
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
    void inverseKinematics(float pose[NUM_LEGS]){
      float euler_angles[3] = {pose[5],pose[4],pose[3]};
      Kinematics::rotationAsEuler(euler_angles);
      BLA::Matrix<3,1> pose_as_matrix = {pose[0],pose[1],pose[2]};
      for(int i = 0;i < NUM_LEGS ;i++){
        int j = (i % 2 == 1) ? -1 : 1;
        Kinematics::base_attachments[i] = {base_radius * cos(angles_rad[i] + j * phi_base / 2.0f),base_radius * sin(angles_rad[i] + j * phi_base / 2.0f), 0.0f};
        Kinematics::platform_attachments[i] = {platform_radius * cos(angles_rad[i] + j * phi_platform / 2.0f),platform_radius * sin(angles_rad[i] + j * phi_platform / 2.0f), 0.0f};
        Kinematics::legs[i] = pose_as_matrix + (r * Kinematics::platform_attachments[i]) - Kinematics::base_attachments[i]; // Leg vectors
        Kinematics::legs_length[i] = Kinematics::norm(legs[i]);
        Kinematics::unit_legs[i] = legs[i] * (1/Kinematics::legs_length[i]);
        Kinematics::platform_attachments[i] = Kinematics::legs[i] + Kinematics::base_attachments[i]; // Moving platform with respect to base
      }
    }

    void calculateJacobian() {
      for(int i = 0;i < NUM_LEGS; i++) {
        Matrix<3,1,float> cross_vector = BLA::CrossProduct(Kinematics::r*Kinematics::platform_attachments[i], Kinematics::unit_legs[i]);
        Kinematics::jacobian(i,0) = unit_legs[i](0);
        Kinematics::jacobian(i,1) = unit_legs[i](1);
        Kinematics::jacobian(i,2) = unit_legs[i](2);
        Kinematics::jacobian(i,3) = cross_vector(0);
        Kinematics::jacobian(i,4) = cross_vector(1);
        Kinematics::jacobian(i,5) = cross_vector(2);
      }
    }


    /*
    * Constructor
    */
    Kinematics(float base_radius, float platform_radius, float phi_base, float phi_platform) : base_radius(base_radius), platform_radius(platform_radius), phi_base(phi_base), phi_platform(phi_platform) {
      
    }

    //debug-method
    BLA::Matrix<6,1> runInverseKinematics(float pose[NUM_LEGS]) {
      Kinematics::inverseKinematics(pose);
      BLA::Matrix<6,1> m = {Kinematics::legs_length[0],Kinematics::legs_length[1],Kinematics::legs_length[2],Kinematics::legs_length[3],Kinematics::legs_length[4],Kinematics::legs_length[5]};
      return m;
    }

    BLA::Matrix<6,6> getJacobian() {
      return jacobian;
    }

    BLA::Matrix<6,6> getUnitLegs() {
      BLA::Matrix<3,6> m;
      for(int i = 0; i < NUM_LEGS;i++) {
        m(1,i) = Kinematics::unit_legs[] 
      }
    }

};

#endif