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

    void setUpPlatform() {
      for(int i = 0; i < NUM_LEGS; i++) {
        int j = (i % 2 == 1) ? -1 : 1;
        base_attachments[i] = {base_radius * cos(angles_rad[i] + j * phi_base / 2.0f), base_radius * sin(angles_rad[i] + j * phi_base / 2.0f), 0.0f};
        platform_attachments[i] = {platform_radius * cos(angles_rad[i] + j * phi_platform / 2.0f), platform_radius * sin(angles_rad[i] + j * phi_platform / 2.0f), 0.0f};
      }
    }

    float norm(BLA::Matrix<3,1> leg_vec) {
      return sqrt(leg_vec(0)*leg_vec(0) + leg_vec(1)*leg_vec(1) + leg_vec(2)*leg_vec(2)); 
    }

    BLA::Matrix<3,1> cross(BLA::Matrix<3,1> a, BLA::Matrix<3,1> b) {
      return {a(1)*b(2) - a(2)*b(1), a(2)*b(0) - a(0)*b(2), a(0)*b(1) - a(1)*b(0)};
    }

    void rotationAsEuler(float angles[DIM]) {
      float cz = cos(angles[0]), sz = sin(angles[0]);  // Z-axis
      float cy = cos(angles[1]), sy = sin(angles[1]);  // Y-axis
      float cx = cos(angles[2]), sx = sin(angles[2]);  // X-axis

      // Column-major initialization
      r = {
        // Column 0
        cy*cz,
        cz*sx*sy - cx*sz,
        sx*sz + cx*cz*sy,
        // Column 1
        cy*sz,
        cx*cz + sx*sy*sz,
        cx*sy*sz - cz*sx,
        // Column 2
        -sy,
        cy*sx,
        cx*cy
      };
    }

    void inverseKinematics(float pose[NUM_LEGS]){
      float euler_angles[3] = {pose[5], pose[4], pose[3]};
      rotationAsEuler(euler_angles);
      BLA::Matrix<3,1> pose_as_matrix = {pose[0], pose[1], pose[2]};
      for(int i = 0; i < NUM_LEGS; i++){
        legs[i] = pose_as_matrix + (r * platform_attachments[i]) - base_attachments[i];
        legs_length[i] = norm(legs[i]);
        unit_legs[i] = legs[i] / legs_length[i];
      }
    }

    void calculateJacobian() {
      for(int i = 0; i < NUM_LEGS; i++) {
        Matrix<3,1> cross_vector = cross(r * platform_attachments[i], unit_legs[i]);
        jacobian(i,0) = unit_legs[i](0);
        jacobian(i,1) = unit_legs[i](1);
        jacobian(i,2) = unit_legs[i](2);
        jacobian(i,3) = cross_vector(0);
        jacobian(i,4) = cross_vector(1);
        jacobian(i,5) = cross_vector(2);
      }
    }

  public:
    Kinematics(float base_radius, float platform_radius, float phi_base, float phi_platform) 
      : base_radius(base_radius), platform_radius(platform_radius), phi_base(phi_base), phi_platform(phi_platform) {
      setUpPlatform(); // Initialize attachments once
    }

    BLA::Matrix<6,1> runInverseKinematics(float pose[NUM_LEGS]) {
      inverseKinematics(pose);
      calculateJacobian(); // Ensure Jacobian is updated
      return {legs_length[0], legs_length[1], legs_length[2], legs_length[3], legs_length[4], legs_length[5]};
    }

    BLA::Matrix<6,6> getJacobian() {
        return jacobian;
      }
  
      BLA::Matrix<3,6> getUnitLegs() {
        BLA::Matrix<3,6> m;
        for(int i = 0; i < NUM_LEGS;i++) {
          m(0,i) = Kinematics::unit_legs[i](0); 
          m(1,i) = Kinematics::unit_legs[i](1); 
          m(2,i) = Kinematics::unit_legs[i](2); 
        }
        return m;
      }
  
      BLA::Matrix<6,3> getLegs() {
        BLA::Matrix<6,3> m;
        for(int i = 0;i < NUM_LEGS; i++) {
          m(i,0) = Kinematics::legs[i](0); 
          m(i,1) = Kinematics::legs[i](1); 
          m(i,2) = Kinematics::legs[i](2);
        }
        return m;
      }
  
      BLA::Matrix<3,3> getR() {
        return Kinematics::r;
      }
};

#endif