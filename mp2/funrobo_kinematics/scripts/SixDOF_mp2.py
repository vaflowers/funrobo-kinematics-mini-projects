from math import *
import math
import numpy as np
import funrobo_kinematics.core.utils as ut
from funrobo_kinematics.core.visualizer import Visualizer, RobotSim
from funrobo_kinematics.core.arm_models import (
    TwoDOFRobotTemplate, ScaraRobotTemplate, FiveDOFRobotTemplate, KinovaRobotTemplate
)


class Kinova(KinovaRobotTemplate):
    def __init__(self):
        super().__init__()

    
    def calc_forward_kinematics(self, joint_values: list, radians=True):
        """
        Calculate Forward Kinematics (FK) based on the given joint angles.

        Args:
            joint_values (list): Joint angles (in radians if radians=True, otherwise in degrees).
            radians (bool): Whether the input angles are in radians (default is False).
        """
        curr_joint_values = joint_values.copy()

        if not radians: # Convert degrees to radians if the input is in degrees
            curr_joint_values = [np.deg2rad(theta) for theta in curr_joint_values]

        # Ensure that the joint angles respect the joint limits
        for i, theta in enumerate(curr_joint_values):
            curr_joint_values[i] = np.clip(theta, self.joint_limits[i][0], self.joint_limits[i][1])
        
        # DH parameters for each joint
        DH = np.zeros((self.num_dof + 1, 4))
        DH[0] = [0, 0, 0, pi]
        DH[1] = [curr_joint_values[0], -self.l1 - self.l2, 0, pi/2]
        DH[2] = [curr_joint_values[1] - pi/2, 0, self.l3, pi]
        DH[3] = [curr_joint_values[2] - pi/2, 0, 0, pi/2]
        DH[4] = [curr_joint_values[3], -self.l4 - self.l5, 0, -pi/2]
        DH[5] = [curr_joint_values[4], 0, 0, pi/2]
        DH[6] = [curr_joint_values[5], -self.l6 - self.l7, 0, pi]
        

        # Compute the transformation matrices
        Hlist = [ut.dh_to_matrix(dh) for dh in DH]

        # Precompute cumulative transformations to avoid redundant calculations
        H_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            H_cumulative.append(H_cumulative[-1] @ Hlist[i])

        # Calculate EE position and rotation
        H_ee = H_cumulative[-1]  # Final transformation matrix for EE

        # Set the end effector (EE) position
        ee = ut.EndEffector()
        ee.x, ee.y, ee.z = (H_ee @ np.array([0, 0, 0, 1]))[:3]
        
        # Extract and assign the RPY (roll, pitch, yaw) from the rotation matrix
        rpy = ut.rotm_to_euler(H_ee[:3, :3])
        ee.rotx, ee.roty, ee.rotz = rpy[0], rpy[1], rpy[2]

        return ee, Hlist
    
    def calc_inverse_kinematics(self, ee, joint_values: list, radians=True, soln = 0):
           
            # position and rotation of end effector
            p_ee = np.array([ee.x, ee.y, ee.z])
            r_ee = ut.euler_to_rotm([ee.rotx, ee.roty, ee.rotz])
            #wrist center position
            piv_wrist = p_ee - (self.l6 + self.l7) * (r_ee @ np.array([0, 0, 1]))
            wrist_x, wrist_y, wrist_z = piv_wrist[0], piv_wrist[1], piv_wrist[2]
            # calculate theta1
            solutions = []
            theta_1_opts = [(-atan2(wrist_y, wrist_x), sqrt(wrist_x**2 + wrist_y**2)),
                            (-atan2(-wrist_y, -wrist_x), -sqrt(wrist_x**2 + wrist_y**2))]
            for theta1, r_val in theta_1_opts:
                # triangle
                r = r_val
                s = wrist_z - (self.l1 + self.l2)
                L_sq = r**2 + s**2
                L = sqrt(L_sq)
               
                l_ste = self.l3
                l_etw = self.l4 + self.l5
               
                # Law of Cosines
                # ensure target is reachable
                if L > (l_ste + l_etw) or L < abs(l_ste - l_etw):
                    continue
                   
                gamma = atan2(s, r)
               
                cos_alpha = np.clip((l_ste**2 + L_sq - l_etw**2) / (2 * l_ste * L), -1.0, 1.0)
                alpha = acos(cos_alpha)
               
                cos_beta = np.clip((l_ste**2 + l_etw**2 - L_sq) / (2 * l_ste * l_etw), -1.0, 1.0)
                beta = acos(cos_beta)
           
                # theta 3 solutions
                for theta_2_candidate, theta_3_candidate in [(np.pi / 2 - (gamma - alpha), np.pi - beta),
                                                             (np.pi / 2 - (gamma + alpha), -(np.pi - beta))]:
                    theta2 = theta_2_candidate
                    theta3 = theta_3_candidate
                   
                    # solve for wrist angles
                    q_temp = [theta1, theta2, theta3, 0, 0, 0]
                   
                    H_cumulative, _ = self.compute_transformation_matrices(q_temp)
                    R3 = H_cumulative[4][:3, :3]
                   
                    R36 = R3.T @ r_ee
                   
                    c5 = -R36[2, 2]
                    s5_mag = sqrt(np.clip(1 - c5**2, 0.0, 1.0))
                   
                    for s5 in [s5_mag, -s5_mag]:
                        theta5 = atan2(s5, c5)
                       
                        theta4 = atan2(-R36[1, 2], -R36[0, 2])
                        theta6 = atan2(-R36[2, 1], -R36[2, 0])
                       
                        candidate_q = [theta1, theta2, theta3, theta4, theta5, theta6]
                        candidate_q = [self.normalized_angle(q) for q in candidate_q]
                       
                        # set limits
                        if ut.check_joint_limits(candidate_q, self.joint_limits):
                            solutions.append(candidate_q)

            def calc_error(q):
                ee_curr, _ = self.calc_forward_kinematics(q)
                return np.linalg.norm(np.array([ee.x - ee_curr.x, ee.y - ee_curr.y, ee.z - ee_curr.z]))
           
            solutions.sort(key=calc_error)
            if not solutions:
                return np.zeros(6)
            if soln < len(solutions):
                return solutions[soln]
            return solutions[0]

    def normalized_angle(self, angle):
        """Normalize an angle to the range (-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi  
        
    
        
        
    


if __name__ == "__main__":
    model = Kinova()
    robot = RobotSim(robot_model=model)
    viz = Visualizer(robot=robot)
    viz.run()