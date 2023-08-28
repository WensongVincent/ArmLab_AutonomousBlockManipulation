"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""
import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm

#********** MODIFY VARIABLES HERE FOR OPTIMAL KINEMATICS ***********
kinematic_variables = {
    'pre_pick_height': 30,
    'pick_small': -15,
    'pick_big': -25,
    'straight_wrist_raised' : 5,
    'straight_wrist_dist' : 100,
    # For blocks that are less than 200mm from center of base, relax pickup height
    "close_pick_radius": 280,
    'close_pick_spacing': 6,

    'pre_place': 55,
    'place_big' : 18,
    'place_small' : 0,
        
    'post_safe': 100,
    # Increasing this value will, in an elbow up configuration, *decrease* the shoulder angle = raise UP the shoulder (theta2)
    'shoulder_raise_gain': 1.5,
    # Increasing this value will, in an elbow up configuration, *increase* the elbow angle = elbow will point down more (-theta3)
    'elbow_raise_gain': 0.8,
    # Slight angle offset to base angle due to arm tilted slightly to the right
    'base_angle_offset': 0.058/2
}

# link 2, extra bit on servo motor for link 2, diagonal of link 2, l3, dist to ee
base_link = 103.91
link2_vert = 200.0
link2_horiz = 50.0
link2 = 205.73
link3 = 200.0

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def MatrixExp6(se3mat):
    """Matrix exponential of se3 representation"""
    se3mat = np.array(se3mat)
    omgtheta = so3ToVec(se3mat[0: 3, 0: 3])
    if NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3), se3mat[0: 3, 3]], [[0, 0, 0, 1]]]
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = se3mat[0: 3, 0: 3] / theta
        return np.r_[np.c_[MatrixExp3(se3mat[0: 3, 0: 3]),
                           np.dot(np.eye(3) * theta \
                                  + (1 - np.cos(theta)) * omgmat \
                                  + (theta - np.sin(theta)) \
                                    * np.dot(omgmat,omgmat),
                                  se3mat[0: 3, 3]) / theta],
                     [[0, 0, 0, 1]]]


def so3ToVec(so3mat):
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])


def NearZero(z):
    return abs(z) < 1e-6


def AxisAng3(expc3):
    return (Normalize(expc3), np.linalg.norm(expc3))


def Normalize(V):
    return V / np.linalg.norm(V)


def MatrixExp3(so3mat):
    omgtheta = so3ToVec(so3mat)
    if NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)


def VecTose3(V):
    """se3 representation of spatial velocity vector"""
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1, 4))]


def VecToso3(omg):
    """Returns skew-symmetric matrix representation of omega"""
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])


def get_components_from_T(T):
    """HELPER function, gets rotation matrix and position vector from homogeneous transform matrix
    """
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    R31 = T[2,0]
    R32 = T[2,1]
    R33 = T[2,2]

    R21 = T[1,0]
    R11 = T[0,0]

    R12 = T[0,1]
    R13 = T[0,2]

    if(np.abs(R31)!=1):
        th1 = -np.arcsin(R31)
        psi = np.arctan2(R32/np.cos(th1),R33/np.cos(th1))
        phi = np.arctan2(R21/np.cos(th1),R11/np.cos(th1))
    else:
        phi = 0
        if (R31 ==-1):
            th1 = np.pi/2
            psi = phi + np.arctan2(R12,R13)
        else:
            th1 = -np.pi/2
            psi = phi + np.arctan2(-R12,-R13)
    pose = np.array([T[0,3]/1000,T[1,3]/1000,T[2,3]/1000,th1,psi,phi])
    return pose


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    end_eff_frame = m_mat
    for i in range(len(joint_angles) - 1, -1, -1):
        end_eff_frame = np.dot(MatrixExp6(VecTose3(s_lst[:, i] * joint_angles[i])), end_eff_frame)
    return end_eff_frame


# def IK_geometric(dh_params, pose):
def IK_geometric(wrist_c, gamma, block_theta):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      wrist_c     Coordinates of wrist center
    @param      gamma       The desired angle from horizontal for the wrist
    @param      block_theta The desired orientation for the gripper to pick up the block

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    x_c = wrist_c[0]  # wrist center given as [x, y, z]
    y_c = wrist_c[1]
    z_c = wrist_c[2] - base_link # offset of base, so origin is at "zero"
    x_y_dist = np.sqrt(x_c**2 + y_c**2)
    
    # Base angle, aligned with y-axis
    theta1 = -np.arctan2(x_c, y_c)
    theta1_sol2 = np.pi + theta1   

    # Angle between link 1 and link 2
    alpha = np.arccos((link2**2 + link3**2 - (x_y_dist**2 + z_c**2)) / (2 * link2 * link3))
    # Angle between wrist center and link 1
    beta = np.arccos((x_y_dist**2 + z_c**2 + link2**2 - link3**2) / (2 * link2 * np.linalg.norm([x_y_dist, z_c])))

    offset_theta = np.arctan(link2_horiz,link2_vert) * 1.25  # atan(50/200)
    # Elbow down configuration
    angle_from_horiz = np.arctan2(z_c, x_y_dist)
    theta2_down = np.pi/2 - (angle_from_horiz - beta) - (offset_theta * kinematic_variables["shoulder_raise_gain"])
    theta3_down = np.pi/2 + (np.pi - alpha) - (offset_theta * kinematic_variables["elbow_raise_gain"])
    theta4_down = gamma - (angle_from_horiz - beta) - (np.pi - alpha)

    # Elbow up configuration
    theta2_up = np.pi/2 - (angle_from_horiz + beta) - offset_theta
    theta3_up = np.pi/2 - (np.pi - alpha) - (offset_theta * kinematic_variables["elbow_raise_gain"])    
    theta4_up = gamma - (angle_from_horiz + beta) - (-1*(np.pi - alpha))

    if gamma == 0:
        # For straight angle wrist, set other joint to 0
        theta5 = 0
    else:
        # For the angle of theta5, aka the orientation of the gripper, start at 0 and 
        # remove the angle from the base to become parallel with the x axis
        # then, using the orientation from block detection, match that angle
        theta5 = theta1 + block_theta
    
    theta1 = theta1 + kinematic_variables["base_angle_offset"]
    all_poss_joint_configs = np.array([[theta1, theta2_up, theta3_up, theta4_up, theta5],
                                        [theta1_sol2, theta2_up, theta3_up, theta4_up, theta5],
                                        [theta1, theta2_down, theta3_down, theta4_down, theta5],
                                        [theta1_sol2, theta2_down, theta3_down, theta4_down, theta5]])
    return all_poss_joint_configs


#######  Unused template functions ###################################

def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    pass


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    pass


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    pass