#ifndef KINEMATICS
#define KINEMATICS

#include <BasicLinearAlgebra.h>
using namespace BLA;

const int NUM_LEGS = 6;
const int DIM = 3;
const double angles_rad[6] = {0.0, 2.0943951, 2.0943951, 4.1887902, 4.1887902, 0.0}; // BASE ANGLES USED FOR PLATFORM

// Constants for iteration
const int MAX_INTERATIONS PROGMEM = 100;
const double TOLERANCE PROGMEM = 1e-3;
const double ALPHA_POS PROGMEM = 0.2;
const double ALPHA_ROT PROGMEM = 0.5;

class Kinematics {
  public:
    /*
    * PLATFORM PARAMS
    */
    double base_radius; // RADIUS OF BASE
    double platform_radius; // RADIUS OF PLATFORM
    double phi_base; // RADIANS
    double phi_platform; // RADIANS

    /*
    * PLATFORM MATRICES
    */
    BLA::Matrix<3,1,double> base_attachments[6]; // COORDINATES OF BASE ATTACHMENTS POINTS
    BLA::Matrix<3,1,double> platform_attachments[6]; // COORDINATES OF PLATFORM ATTACHMENT POINTS
    BLA::Matrix<3,1,double> legs[6]; // LEG VECTORS
    BLA::Matrix<3,1,double> unit_legs[6]; // UNIT LEG VECTORS FOR JACOBIAN
    double legs_length[6]; // L2 NORM OF LEGS

    /*
    * TRANSFORMATION MATRICES
    */
    BLA::Matrix<3,3,double> r; // ROTATION MATRIX
    BLA::Matrix<3,3,double> r_delta;
    BLA::Matrix<4,4,double> est_pose;
    BLA::Matrix<4,4,double> delta_T;

    /*
    * JACOBIAN MATRICES
    */
    BLA::Matrix<6,6,double> jacobian;
    BLA::Matrix<6,6,double> jacobian_transpose;
    BLA::Matrix<6,6,double> j_star; 

    void setUpPlatform() {
      for(int i = 0; i < NUM_LEGS; i++) {
        int j = (i % 2 == 1) ? -1 : 1;
        base_attachments[i] = {base_radius * cos(angles_rad[i] + j * phi_base / 2.0f), base_radius * sin(angles_rad[i] + j * phi_base / 2.0), 0.0};
        platform_attachments[i] = {platform_radius * cos(angles_rad[i] + j * phi_platform / 2.0f), platform_radius * sin(angles_rad[i] + j * phi_platform / 2.0), 0.0};
      }
    }

    double norm(BLA::Matrix<3,1,double> leg_vec) {
      return sqrt(leg_vec(0)*leg_vec(0) + leg_vec(1)*leg_vec(1) + leg_vec(2)*leg_vec(2)); 
    }

    // double norm(double leg_vec[6]) {
    //   double cubesum = 0;
    //   for(int i = 0; i < NUM_LEGS; i++) {
    //     cubesum += leg_vec[i]*leg_vec[i];
    //   }
    //   return sqrt(cubesum);
    // }

    BLA::Matrix<3,1,double> cross(BLA::Matrix<3,1,double> a, BLA::Matrix<3,1,double> b) {
      return {a(1)*b(2) - a(2)*b(1), a(2)*b(0) - a(0)*b(2), a(0)*b(1) - a(1)*b(0)};
    }


    //, BLA::Matrix<3,3,double> rot
    void rotationAsEuler(double angles[DIM]) {
      double cz = cos(angles[0]), sz = sin(angles[0]);  // Z-axis
      double cy = cos(angles[1]), sy = sin(angles[1]);  // Y-axis
      double cx = cos(angles[2]), sx = sin(angles[2]);  // X-axis

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
    
    /*
    * Returns the lengths of the legs based on a pose using a vector-loop equation.
    */
    void inverseKinematics(double pose[NUM_LEGS]){
      double euler_angles[3] = {pose[5], pose[4], pose[3]};
      rotationAsEuler(euler_angles);
      BLA::Matrix<3,1,double> pose_as_matrix = {pose[0], pose[1], pose[2]};
      for(int i = 0; i < NUM_LEGS; i++){
        legs[i] = pose_as_matrix + (r * platform_attachments[i]) - base_attachments[i];
        legs_length[i] = norm(legs[i]);
        unit_legs[i] = legs[i] / legs_length[i];
      }
    }


    // maybe preallocate
    void calculateJacobian() {
      for(int i = 0; i < NUM_LEGS; i++) {
        Matrix<3,1,double> cross_vector = cross(r * platform_attachments[i], unit_legs[i]);
        jacobian(i,0) = unit_legs[i](0);
        jacobian(i,1) = unit_legs[i](1);
        jacobian(i,2) = unit_legs[i](2);
        jacobian(i,3) = cross_vector(0);
        jacobian(i,4) = cross_vector(1);
        jacobian(i,5) = cross_vector(2);
      }
    }

  public:
    double position[6] = {0,0,0,0,0,0};
    /*
    * CONSTRUCTOR
    */
    Kinematics(double base_radius, double platform_radius, double phi_base, double phi_platform) 
      : base_radius(base_radius), platform_radius(platform_radius), phi_base(phi_base), phi_platform(phi_platform) {
      setUpPlatform(); // Initialize attachments once
    }

    /*
    * Returns the position of the platform based on the lengths of the legs using Newton-Raphson.
    */
    // bool forwardKinematics(float initial_guess[6],float measured_lengths[6]) {
    //   // ROTATION based on guess
    //   double euler_angles[3] = {pose[5], pose[4], pose[3]};
    //   rotationAsEuler(euler_angles,r);
      
    //   /*
    //   * Building homogenus transformation matrix
    //   */ 
    //   for(int i = 0;i < DIM; i++) {
    //     for(int j = 0; j < DIM; j++) {
    //       Kinematics::est_pose(i,j) = r(i,j);
    //     }
    //     Kinematics::est_pose(i,3) = pose[i];
    //   }
    //   Kinematics::est_pose(3,0) = 0;
    //   Kinematics::est_pose(3,1) = 0;
    //   Kinematics::est_pose(3,2) = 0;
    //   Kinematics::est_pose(3,3) = 1;

    //   // Iteration variables
    //   double delta_lengths[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
    //   int count = 0;
    //   double error = 1 + TOLERANCE;
    //   double legs_est[6];

    //   while(error > TOLERANCE && count < 100) { //maybe change to 50?
    //     // define new variables
    //     Kinematics::inverseKinematics();
    //     legs_est = Kinematics::legs_length;
    //     for(int i = 0; i < NUM_LEGS; i++) {
    //       delta_lengths[i] = measured_lengths[i]-legs_est[i];
    //     }
    //     error = Kinematics::norm(delta_lengths);
    //     Kinematics::calculateJacobian();

    //   }

    // }



    BLA::Matrix<6,1,double> runInverseKinematics(double pose[NUM_LEGS]) {
      inverseKinematics(pose);
      calculateJacobian(); // Ensure Jacobian is updated
      return {legs_length[0], legs_length[1], legs_length[2], legs_length[3], legs_length[4], legs_length[5]};
    }

    BLA::Matrix<6,6,double> getJacobian() {
        return jacobian;
      }
  
      BLA::Matrix<3,6,double> getUnitLegs() {
        BLA::Matrix<3,6,double> m;
        for(int i = 0; i < NUM_LEGS;i++) {
          m(0,i) = Kinematics::unit_legs[i](0); 
          m(1,i) = Kinematics::unit_legs[i](1); 
          m(2,i) = Kinematics::unit_legs[i](2); 
        }
        return m;
      }
  
      BLA::Matrix<6,3,double> getLegs() {
        BLA::Matrix<6,3,double> m;
        for(int i = 0;i < NUM_LEGS; i++) {
          m(i,0) = Kinematics::legs[i](0); 
          m(i,1) = Kinematics::legs[i](1); 
          m(i,2) = Kinematics::legs[i](2);
        }
        return m;
      }
  
      BLA::Matrix<3,3,double> getR() {
        return Kinematics::r;
      }
};

#endif