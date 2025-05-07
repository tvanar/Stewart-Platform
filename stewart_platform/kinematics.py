import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class StewartPlatform:

    def __init__(self):
        self.base_base = np.array([[92,52.5, 0],
                                   [92,-52.5, 0],
                                   [0, -92, 0],
                                   [-92, -52.5, 0],
                                   [-92, 52.5, 0],
                                   [0, 92, 0]])
        

        self.platform_platform = np.array([[46.0,13.374358, 0],
                                           [46.0,-13.374358, 0],
                                           [-11.417466, -46.524348, 0],
                                           [-33.149989, -34.582534, 0],
                                           [-33.149989, 34.582534, 0],
                                           [-11.417466, 46.524348, 0],])
        
    ############################################ IK ##############################################
    def inverse_kinematics(self, pose): ## Inverse Kinematics (Vectorial Closing Loop)
        if len(pose) != 6:
            raise ValueError("Pose must be a 6-element list or array")

        self.p_i = np.zeros([6, 3])
        self.b_i = np.zeros([6, 3])
        self.l_i = np.zeros([6, 3])
        self.l_i_unit = np.zeros([6, 3])

        self.pose_vect = pose
        x_p, y_p, z_p, roll, pitch, yaw = pose

        self.r = R.from_euler("ZYX", (yaw, pitch, roll), degrees=True)  #rotation matrix
        angles = np.array([0, 120, 120, 240, 240, 0])

        self.b_i = self.base_base.copy()
        self.p_i = self.platform_platform.copy()
        for i in range(6):
            self.l_i[i, :] = np.array([x_p, y_p, z_p]) + self.r.as_matrix().dot(self.p_i[i]) - self.b_i[i] # legs vectors
            self.l_i_unit[i, :] = self.l_i[i, :] / np.linalg.norm(self.l_i[i, :]) # unit legs vectors
            self.p_i[i] = self.l_i[i, :] + self.b_i[i] # moving platform points wrt base frame

        return self.l_i
    
    def __jacobian(self): # Calculate Jacobian q_dot=Jacobian*x_dot
        if not hasattr(self, 'l_i_unit') or not hasattr(self, 'p_i'):
            raise AttributeError("Run inverse kinematics before calculating the Jacobian")
        self.jacobian = np.zeros([6, 6])
        for i in range(6):
            lastcols = np.cross(self.r.as_matrix().dot(self.p_i[i]), self.l_i_unit[i])
            self.jacobian[i, :3] = self.l_i_unit[i]
            self.jacobian[i, 3:] = lastcols

        return self.jacobian
    
    ############################################ FK ##############################################
    def forward_kinematics(self, starting_pose, lengths_desired): # Newton Method Optimization.
        if not isinstance(starting_pose, (list, np.ndarray)) or len(starting_pose) != 6:
            raise ValueError("Starting pose must be a list or numpy array with 6 elements")
        if not isinstance(lengths_desired, (list, np.ndarray)) or len(lengths_desired) != 6:
            raise ValueError("Lengths desired must be a list or numpy array with 6 elements")

        self.pose_vect = starting_pose
        x_p, y_p, z_p, roll, pitch, yaw = starting_pose
        self.r = R.from_euler("ZYX", (yaw, pitch, roll), degrees=True)

        # Newton Method Parameters
        max_count = 50
        epsilon = 2
        alpha_pos = 0.2
        alpha_rot = 0.01
        # variables
        est_pose_vect = np.copy(starting_pose)
        est_pose = np.zeros([4, 4])
        est_pose[:3, :3] = self.r.as_matrix()
        est_pose[:3, 3] = [x_p, y_p, z_p]
        est_pose[3, 3] = 1
        delta_lengths = np.zeros(6)
        delta_T = np.identity(4)
        error = epsilon + 1
        count = 0
        

        while error > epsilon and count < max_count:
            # find new variables
            legs_est = self.inverse_kinematics(est_pose_vect)
            # print('lengths_est: ',np.linalg.norm(legs_est,axis=1))
            lengths_est = np.linalg.norm(legs_est, axis=1)
            delta_lengths = lengths_desired - lengths_est
            
            J = self.__jacobian()
            # find x_delta with pseudoinverse
            J_T = J.transpose()
            J_star = np.linalg.inv(J_T.dot(J) + 0.0001 * np.identity(6)).dot(J_T)
            x_delta = np.dot(J_star, delta_lengths)
            x_delta_pos = x_delta[:3] * alpha_pos
            x_delta_rot = x_delta[3:] * alpha_rot
            # transform x_delta to delta_T
            r_delta = R.from_euler("ZYX", x_delta_rot)
            delta_T[:3, :3] = r_delta.as_matrix()
            delta_T[:3, 3] = x_delta[:3]
            # move pose
            est_pose = np.dot(delta_T, est_pose)
            # transform est_pose to est_pose_vect
            est_pose_vect[:3] = est_pose[:3, 3]
            self.r_pos = R.from_matrix(est_pose[:3, :3])
            est_pose_vect[3:] = self.r_pos.as_euler("ZYX", degrees=True)
            # find error and update counter
            error = np.linalg.norm(delta_lengths)
            count += 1
        
        # print('num_iters: ',count)
        if count < max_count:
            self.pose_vect = est_pose_vect

        return self.pose_vect