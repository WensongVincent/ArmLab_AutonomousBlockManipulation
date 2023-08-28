"""!
Implements the RXArm class.

The RXArm class contains:

* last feedback from joints
* functions to command the joints
* functions to get feedback from joints
* functions to do FK and IK
* A function to read the RXArm config file

You will upgrade some functions and also implement others according to the comments given in the code.
"""
import numpy as np
from functools import partial
from kinematics import FK_pox, get_pose_from_T, get_components_from_T, IK_geometric, kinematic_variables
import time
import csv
import copy
from builtins import super
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
from interbotix_robot_arm import InterbotixRobot
from interbotix_descriptions import interbotix_mr_descriptions as mrd
from config_parse import *
from sensor_msgs.msg import JointState
import rospy
"""
TODO: Implement the missing functions and add anything you need to support them
"""
""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi


def _ensure_initialized(func):
    """!
    @brief      Decorator to skip the function if the RXArm is not initialized.

    @param      func  The function to wrap

    @return     The wraped function
    """
    def func_out(self, *args, **kwargs):
        if self.initialized:
            return func(self, *args, **kwargs)
        else:
            print('WARNING: Trying to use the RXArm before initialized')

    return func_out


class RXArm(InterbotixRobot):
    """!
    @brief      This class describes a RXArm wrapper class for the rx200
    """
    def __init__(self, dh_config_file=None):
        """!
        @brief      Constructs a new instance.

                    Starts the RXArm run thread but does not initialize the Joints. Call RXArm.initialize to initialize the
                    Joints.

        @param      dh_config_file  The configuration file that defines the DH parameters for the robot
        """
        super().__init__(robot_name="rx200", mrd=mrd)
        
        # Variables from kinematics.py
        self.pre_pick_height = kinematic_variables["pre_pick_height"]
        self.pick_small = kinematic_variables["pick_small"]
        self.pick_big = kinematic_variables["pick_big"]
        self.pre_place = kinematic_variables["pre_place"]
        self.place_big = kinematic_variables["place_big"]
        self.post_safe = kinematic_variables["post_safe"]

        self.joint_names = self.resp.joint_names
        self.num_joints = 5
        # Gripper
        self.gripper_state = True
        # State
        self.initialized = False
        self.holding_block = False
        # Cmd
        self.position_cmd = None
        self.moving_time = 2.0
        self.accel_time = 0.5
        # Feedback
        self.position_fb = None
        self.velocity_fb = None
        self.effort_fb = None
        # DH Params
        self.dh_params = []
        self.dh_config_file = dh_config_file
        if (dh_config_file is not None):
            self.dh_params = RXArm.parse_dh_param_file(dh_config_file)
        #POX params
        # Home configuration of the end-effector
        self.M_matrix = np.array([[1,0,0,0],
                                [0,1,0,408.575],
                                [0,0,1,304.57],
                                [0,0,0,1]])
        # Joint screw axes in the space frame  
        self.S_list = np.array([[0,0,1,   0,0,0],
                  [-1,0,0,  0,-104.57,0],
                  [1,0,0,   0,304.57,-50],
                  [1,0,0,   0,304.57,-250],
                  [0,1,0,   -304.47,0,0]]).T
        self.link2 = 205.73
        self.link3 = 200.0
        self.wrist_to_ee_length = 174.15
        # Constant used to determine when to use a 0 deg (parallel to horiz) wrist vs -90 deg
        self.wrist_straight_perimeter = 1.0

    def initialize(self):
        """!
        @brief      Initializes the RXArm from given configuration file.

                    Initializes the Joints and serial port

        @return     True is succes False otherwise
        """
        self.initialized = False
        # Wait for other threads to finish with the RXArm instead of locking every single call
        rospy.sleep(0.25)
        """ Commanded Values """
        self.position = [0.0] * self.num_joints  # radians
        """ Feedback Values """
        self.position_fb = [0.0] * self.num_joints  # radians
        self.velocity_fb = [0.0] * self.num_joints  # 0 to 1 ???
        self.effort_fb = [0.0] * self.num_joints  # -1 to 1

        # Reset estop and initialized
        self.estop = False
        self.enable_torque()
        self.moving_time = 2.0
        self.accel_time = 0.5
        self.set_gripper_pressure(1.0)
        self.go_to_home_pose(moving_time=self.moving_time,
                             accel_time=self.accel_time,
                             blocking=False)
        self.open_gripper()
        self.initialized = True
        return self.initialized

    def sleep(self):
        self.moving_time = 2.0
        self.accel_time = 1.0
        self.go_to_home_pose(moving_time=self.moving_time,
                             accel_time=self.accel_time,
                             blocking=True)
        self.go_to_sleep_pose(moving_time=self.moving_time,
                              accel_time=self.accel_time,
                              blocking=False)
        self.initialized = False

    def set_positions(self, joint_positions):
        """!
         @brief      Sets the positions.

         @param      joint_angles  The joint angles
         """
        self.set_joint_positions(joint_positions,
                                 moving_time=self.moving_time,
                                 accel_time=self.accel_time,
                                 blocking=False)

    def set_moving_time(self, moving_time):
        self.moving_time = moving_time

    def set_accel_time(self, accel_time):
        self.accel_time = accel_time

    def disable_torque(self):
        """!
        @brief      Disables the torque and estops.
        """
        self.torque_joints_off(self.joint_names)

    def enable_torque(self):
        """!
        @brief      Disables the torque and estops.
        """
        self.torque_joints_on(self.joint_names)

    def get_positions(self):
        """!
        @brief      Gets the positions.

        @return     The positions.
        """
        return self.position_fb

    def get_velocities(self):
        """!
        @brief      Gets the velocities.

        @return     The velocities.
        """
        return self.velocity_fb

    def get_efforts(self):
        """!
        @brief      Gets the loads.

        @return     The loads.
        """
        return self.effort_fb

#   @_ensure_initialized
    def get_ee_pose(self):
        """!
        @brief      TODO Get the EE pose.

        @return     The EE pose as [x, y, z, phi] or as needed.
        """
        
        pose = FK_pox(self.get_joint_positions()[:5], self.M_matrix, self.S_list)
        xyz = get_pose_from_T(pose)
        return list(xyz)

    # @_ensure_initialized
    def get_wrist_pose(self, world_coords, force_straight_wrist=0):
        """!
        @brief      TODO Get the wrist pose.

        @param      pose    Homogeneous tranformation matrix of end effector pose

        @return     The wrist pose as [x, y, z] (removed phi) or as needed AND x/y distance from base center
        """
        rot_mat = np.identity(3)
        pose = np.r_[np.c_[rot_mat, world_coords], [[0, 0, 0, 1]]]
        r_mat, pos_vec = get_components_from_T(pose)
        x_c = pos_vec[0]  # wrist center given as [x, y, z]
        y_c = pos_vec[1]
        x_y_dist = np.sqrt(x_c**2 + y_c**2)
        theta = np.arctan2(y_c, x_c)
        
        if (x_c == 0 and y_c == 0):
            # TODO: test this
            raise Exception("Singularity, need to implement solution")

        if x_y_dist > self.link2 + self.link3 + self.wrist_to_ee_length:
            raise Exception ("Position not within workspace of robot")
                
        # Use 0 deg gamma pose for wrist if distance is some K * (link2 + link3)
        if (x_y_dist > (self.link2 + self.link3) * self.wrist_straight_perimeter) or force_straight_wrist:
            # wrist center is a radial distance away from origin of ee
            pos_vec[0] = pos_vec[0] - (np.cos(theta) * self.wrist_to_ee_length)
            pos_vec[1] = pos_vec[1] - (np.sin(theta) * self.wrist_to_ee_length)
            wrist_pos = pos_vec
            # Add wrist z offset for straight wrist
            wrist_pos[2] = wrist_pos[2] + kinematic_variables["straight_wrist_raised"]
            gamma = 0
        else:
            # wrist center is at a distance along z from origin of ee
            wrist_pos = pos_vec + self.wrist_to_ee_length * np.matmul(r_mat, np.array([0,0,1]).reshape(3,))
            gamma = -np.pi/2
        
        return wrist_pos, gamma
    
    def get_wrist_pose_pick_raised(self, wrist_pos, gamma=-np.pi/2):
        wrist_pos_pick_raised = copy.deepcopy(wrist_pos)
        if gamma == -np.pi/2:
            wrist_pos_pick_raised[2] = wrist_pos_pick_raised[2] + self.pre_pick_height
        else:
            # Pick at a position that is CLOSER to the center of the base radially
            x_c = wrist_pos_pick_raised[0]
            y_c = wrist_pos_pick_raised[1]
            theta = np.arctan2(y_c, x_c)
            wrist_pos_pick_raised[0] = wrist_pos_pick_raised[0] - (np.cos(theta) * kinematic_variables["straight_wrist_dist"])
            wrist_pos_pick_raised[1] = wrist_pos_pick_raised[1] - (np.sin(theta) * kinematic_variables["straight_wrist_dist"])
        return wrist_pos_pick_raised
    
    def get_wrist_pose_pick(self, wrist_pos, block_flag=0):
        """
        Block flag is to determine if big block (0) or small block (1)
        """
        # TODO: change the amount of space depending on the distance from the base
        wrist_pos_pick = copy.deepcopy(wrist_pos)
        x_c = wrist_pos_pick[0]  # wrist center given as [x, y, z]
        y_c = wrist_pos_pick[1]
        x_y_dist = np.sqrt(x_c**2 + y_c**2)
        # Pick block
        if block_flag == 1:
            # Small block
            wrist_pos_pick[2] += self.pick_small
        else:
            # Big block
            wrist_pos_pick[2] += self.pick_big
            if x_y_dist < kinematic_variables["close_pick_radius"]:
                wrist_pos_pick[2] += kinematic_variables["close_pick_spacing"]
        return wrist_pos_pick
    
    def get_wrist_pose_place(self, wrist_pos, block_flag=0, gamma=-np.pi/2):
        wrist_pos_place = copy.deepcopy(wrist_pos)
        # Place, always leave space for block being held
        if block_flag == 1:
            # Small block
            wrist_pos_place[2] = wrist_pos_place[2]
        else:
            # Big block
            wrist_pos_place[2] = wrist_pos_place[2] + self.place_big
        if gamma == 0:
            wrist_pos_place[2] = wrist_pos_place[2] + kinematic_variables["straight_wrist_raised"]
        return wrist_pos_place
    
    def get_wrist_pose_place_raised(self, wrist_pos):
        wrist_pos_place_raised = copy.deepcopy(wrist_pos)
        # Prepare for place
        wrist_pos_place_raised[2] = wrist_pos_place_raised[2] + self.pre_place
        return wrist_pos_place_raised  
    
    def get_wrist_pose_place_safe(self, wrist_pos):
        wrist_safe = copy.deepcopy(wrist_pos)
        wrist_safe[2] = wrist_safe[2] + self.post_safe
        return wrist_safe

    def move_to_ee_pose(self, wrist_pos, gamma=-np.pi/2, block_theta=0):
        # Wrapper for IK
        all_joint_angles = IK_geometric(wrist_pos, gamma=gamma, block_theta=block_theta)
        return all_joint_angles

    def parse_pox_param_file(self):
        """!
        @brief      TODO Parse a PoX config file

        @return     0 if file was parsed, -1 otherwise 
        """
        return -1

    def parse_dh_param_file(self):
        print("Parsing DH config file...")
        parse_dh_param_file(self.dh_config_file)
        print("DH config file parse exit.")
        return dh_params

    def get_dh_parameters(self):
        """!
        @brief      Gets the dh parameters.

        @return     The dh parameters.
        """
        return self.dh_params


class RXArmThread(QThread):
    """!
    @brief      This class describes a RXArm thread.
    """
    updateJointReadout = pyqtSignal(list)
    updateEndEffectorReadout = pyqtSignal(list)

    def __init__(self, rxarm, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      RXArm  The RXArm
        @param      parent  The parent
        @details    TODO: set any additional initial parameters (like PID gains) here
        """
        QThread.__init__(self, parent=parent)
        self.rxarm = rxarm
        rospy.Subscriber('/rx200/joint_states', JointState, self.callback)
        rospy.sleep(0.5)

    def callback(self, data):
        self.rxarm.position_fb = np.asarray(data.position)[0:5]
        self.rxarm.velocity_fb = np.asarray(data.velocity)[0:5]
        self.rxarm.effort_fb = np.asarray(data.effort)[0:5]
        self.updateJointReadout.emit(self.rxarm.position_fb.tolist())
        self.updateEndEffectorReadout.emit(self.rxarm.get_ee_pose())
        #for name in self.rxarm.joint_names:
        #    print("{0} gains: {1}".format(name, self.rxarm.get_motor_pid_params(name)))
        if (__name__ == '__main__'):
            print(self.rxarm.position_fb)

    def run(self):
        """!
        @brief      Updates the RXArm Joints at a set rate if the RXArm is initialized.
        """
        while True:

            rospy.spin()


if __name__ == '__main__':
    rxarm = RXArm()
    print(rxarm.joint_names)
    armThread = RXArmThread(rxarm)
    armThread.start()
    try:
        joint_positions = [-1.0, 0.5, 0.5, 0, 1.57]
        rxarm.initialize()

        rxarm.go_to_home_pose()
        rxarm.set_gripper_pressure(0.5)
        rxarm.set_joint_positions(joint_positions,
                                  moving_time=2.0,
                                  accel_time=0.5,
                                  blocking=True)
        rxarm.close_gripper()
        rxarm.go_to_home_pose()
        rxarm.open_gripper()
        rxarm.sleep()

    except KeyboardInterrupt:
        print("Shutting down")
