"""!
The state machine that implements the logic.
"""
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner>
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = [
            [-np.pi/2,       -0.5,      -0.3,            0.0,       0.0],
            [0.75*-np.pi/2,   0.5,      0.3,      0.0,       np.pi/2],
            [0.5*-np.pi/2,   -0.5,     -0.3,     np.pi / 2,     0.0],
            [0.25*-np.pi/2,   0.5,     0.3,     0.0,       np.pi/2],
            [0.0,             0.0,      0.0,         0.0,     0.0],
            [0.25*np.pi/2,   -0.5,      -0.3,      0.0,       np.pi/2],
            [0.5*np.pi/2,     0.5,     0.3,     np.pi / 2,     0.0],
            [0.75*np.pi/2,   -0.5,     -0.3,     0.0,       np.pi/2],
            [np.pi/2,         0.5,     0.3,      0.0,     0.0],
            [0.0,             0.0,     0.0,      0.0,     0.0]]
        self.recorded_points = []
        self.gripper_state = 0
        
    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()
        
        if self.next_state == "teach":
            self.teach()
            
        if self.next_state == "record":
            self.record()
            
        if self.next_state == "playback":
            self.playback()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()
            
        if self.next_state == "task1":
            self.task1()
            
        if self.next_state == "task2":
            self.task2()
            
        if self.next_state == "task3":
            self.task3()

        if self.next_state == "task4":
            self.task4()

        if self.next_state == "task5":
            self.task5()


    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        
        self.current_state = "execute"
        self.next_state = "idle"
        self.status_message = "State: Execute - Executing motion plan"
        self.moving_time = 1.0
        for pos in self.waypoints:
            self.rxarm.set_positions(pos)
            rospy.sleep(2)
    
    def playback(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        
        self.current_state = "playback"
        self.next_state = "idle"
        self.status_message = "State: Playback - Executing recorded points"
        self.moving_time = 1.0
        # print(self.recorded_points)
        for pos in self.recorded_points:
            joint_pos = pos[:5]
            self.rxarm.set_positions(joint_pos)
            rospy.sleep(1.0)
            # gripper
            gripper_val = pos[5]
            if (gripper_val < 1.0 and not self.gripper_state):
                self.rxarm.close_gripper()
                self.gripper_state = 1
                rospy.sleep(1.0)
            elif(gripper_val >= 1.0 and self.gripper_state):
                self.rxarm.open_gripper()
                self.gripper_state = 0
                rospy.sleep(1.0)

    def teach(self):
        self.current_state = "teach"
        self.next_state = "idle"
        
        self.status_message = "teach"
        self.rxarm.disable_torque()
        self.recorded_points = []
        self.rxarm.set_gripper_operating_mode("pwm")
        
    def record(self):
        self.current_state = "record"
        self.next_state = "idle"
        new_waypoint = self.rxarm.get_joint_positions()
        #print(new_waypoint)
        self.recorded_points.append(new_waypoint)
        print(self.recorded_points)
        time.sleep(0.1)
        print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        success,_,_ = self.camera.apriltag_calibration()
        extrinsic_matrix_naive = self.camera.compute_extrinsic_naive()
        print('extrinsic matrix naive')
        print(extrinsic_matrix_naive)
        print('extrinsic matrix apriltag')
        print(self.camera.extrinsic_matrix)
        # print(success)   
        if success:
            self.camera.calculate_homography_matrix()
            self.camera.cameraCalibrated = True
        self.status_message = "Calibration - Completed Calibration"

    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        rospy.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            rospy.sleep(5)
        self.next_state = "idle"
        
    def task1(self):
        self.initialize_rxarm()
        self.unstack()
        self.rxarm.sleep()
        rospy.sleep(2)
        _ = self.camera.blockDetector() 
        block_locations = self.camera.block_loc_xyz
        no_big = 0
        no_small = 0
        change = True
        while change:
            _ = self.camera.blockDetector()
            block_locations = self.camera.block_loc_xyz
            change = False
            self.initialize_rxarm()
            for block in block_locations:
                block_info = block[1]
                world_coords = [block_info[0],block_info[1],block_info[2]]
                if block_info[1] > 0:
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    
                    if block_info[5] == "big":
                        # Set block flag to 0 for big block
                        self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 0)
                        world_coords = [100+no_big*50,-75,0]
                        no_big += 1
                        change = True
                    elif block_info[5] == "small":
                        # Set block flag to 1 for small block
                        self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 1)
                        world_coords = [-100-no_small*50,-75,0]
                        no_small += 1
                        change = True
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    self.place_at_position(wrist_pos, gamma=gamma, block_theta=np.pi/2)
            self.rxarm.sleep()
            rospy.sleep(2)
     
                
    def task2(self):
        
        no_big = 0
        no_small = 0
        self.initialize_rxarm()
        self.unstack()
        while no_big + no_small < 9:
            self.rxarm.sleep()
            _ = self.camera.blockDetector()
            block_locations = self.camera.block_loc_xyz
            for block in block_locations:
                block_info = block[1]
                world_coords = [block_info[0],block_info[1],block_info[2]]
                if block_info[1] > 0:
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    
                    if block_info[5] == "big":
                        # Set block flag to 0 for big block
                        self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 0)
                        world_coords = [200,-75,0+no_big*35]
                        no_big += 1
                        wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                        self.place_at_position(wrist_pos, gamma=gamma, block_theta=np.pi/2, block_flag= 0)
                    elif block_info[5] == "small":
                        # Set block flag to 1 for small block
                        self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 1)
                        world_coords = [-200,-75,0+no_small*25]
                        no_small += 1
                        wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                        self.place_at_position(wrist_pos, gamma=gamma, block_theta=-np.pi/2, block_flag = 1)    
            
        
    def task3(self):
        self.current_state = "Task 3"
        _ = self.camera.blockDetector()
        block_locations = self.camera.block_loc_xyz
        no_big_blocks = {"red":0, "orange":0, "yellow":0, "green":0, "blue":0, "violet":0}
        no_small_blocks = {"red":0, "orange":0, "yellow":0, "green":0, "blue":0, "violet":0}
        self.initialize_rxarm()
        self.unstack()
        
        
        self.initialize_rxarm()
        change = True
        while change:
            change = False
            self.rxarm.sleep()
            rospy.sleep(2)
            _ = self.camera.blockDetector()
            block_locations = self.camera.block_loc_xyz
            for block_location in block_locations:
                block_info = block_location[1]
                block_coord = [block_info[0],block_info[1],block_info[2]]
                block_color = block_info[4]
                block_size = block_info[5]
                if block_info[1] > 0:
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(block_coord)
                    if block_size == "big":
                        a = self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 0)
                        print(a) 
                        if block_color == "red":
                            block_coord = [100,-75,no_big_blocks["red"]*35]
                            # no_big_blocks["red"] += 1
                        elif block_color == "orange":
                            block_coord = [150,-75,no_big_blocks["orange"]*35]
                            # no_big_blocks["orange"] += 1
                        elif block_color == "yellow":
                            block_coord = [200,-75,no_big_blocks["yellow"]*35]
                            # no_big_blocks["yellow"] += 1
                        elif block_color == "green":
                            block_coord = [250,-75,no_big_blocks["green"]*35]
                            # no_big_blocks["green"] += 1
                        elif block_color == "blue":
                            block_coord = [300,-75,no_big_blocks["blue"]*35]
                            # no_big_blocks["blue"] += 1
                        elif block_color == "violet":
                            block_coord = [350,-75,no_big_blocks["violet"]*35]
                            # no_big_blocks["violet"] += 1
                        wrist_pos, gamma = self.rxarm.get_wrist_pose(block_coord)
                        print("drop")
                        self.place_at_position_task345(wrist_pos, gamma=gamma, block_theta=np.pi/2, block_flag = 0)
                        change = True
                    elif block_size == "small":
                        self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 1)
                        if block_color == "red":
                            block_coord = [-100,-75,no_small_blocks["red"]*35]
                            # no_small_blocks["red"] += 1
                        elif block_color == "orange":
                            block_coord = [-150,-75,no_small_blocks["orange"]*35]
                            # no_small_blocks["orange"] += 1
                        elif block_color == "yellow":
                            block_coord = [-200,-75,no_small_blocks["yellow"]*35]
                            # no_small_blocks["yellow"] += 1
                        elif block_color == "green":
                            block_coord = [-250,-75,no_small_blocks["green"]*35]
                            # no_small_blocks["green"] += 1
                        elif block_color == "blue":
                            block_coord = [-300,-75,no_small_blocks["blue"]*35]
                            # no_small_blocks["blue"] += 1
                        elif block_color == "violet":
                            block_coord = [-350,-75,no_small_blocks["violet"]*35]
                            # no_small_blocks["violet"] += 1
                        wrist_pos, gamma = self.rxarm.get_wrist_pose(block_coord)
                        self.place_at_position_task345(wrist_pos, gamma=gamma, block_theta=-np.pi/2, block_flag = 1)
                        change = True
        self.rxarm.sleep()
        self.next_state = "idle"
    
    def task4(self):
        c_order = ["red","orange","yellow","green","blue","violet","NA"]
        s_stack = 0
        b_stack = 0
        complete = False
        self.initialize_rxarm()
        self.unstack()
        # no_big = 0
        # no_small = 0
        # b_loc = [[150,-25,0],[150,-125,0],[200,-75,0],[250,-25,0],[250,-125,0],[300,-75,0]]
        # s_loc = [[-150,-25,0],[-150,-125,0],[-200,-75,0],[-250,-25,0],[-250,-125,0],[-300,-75,0]]
        # while no_big + no_small < 6:
        #     self.rxarm.sleep()
        #     rospy.sleep(2)
        #     _ = self.camera.blockDetector()
        #     block_locations = self.camera.block_loc_xyz
        #     for block in block_locations:
        #         block_info = block[1]
        #         world_coords = [block_info[0],block_info[1],block_info[2]]
        #         if block_info[1] > 0:
        #             wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    
        #             if block_info[5] == "small":
        #                 # Set block flag to 0 for big block
        #                 self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 0)
        #                 world_coords = b_loc[no_big]
        #                 no_big += 1
        #                 wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
        #                 self.place_at_position(wrist_pos, gamma=gamma, block_theta=np.pi/2, block_flag= 0)
                    # elif block_info[5] == "small":
                    #     # Set block flag to 1 for small block
                    #     self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 1)
                    #     world_coords = s_loc[no_small]
                    #     no_small += 1
                    #     wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    #     self.place_at_position(wrist_pos, gamma=gamma, block_theta=-np.pi/2, block_flag = 1)  
        
        while not complete:
            _ = self.camera.blockDetector()
            block_locations = self.camera.block_loc_xyz
            self.rxarm.sleep()
            rospy.sleep(2)  
            for block in block_locations:
                block_info = block[1]
                if block_info[5] == "big" and block_info[4] == c_order[b_stack]:
                    world_coords = [block_info[0],block_info[1],block_info[2]]
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    self.pick_at_position(wrist_pos, gamma=gamma, block_theta=-np.pi/2, block_flag = 0)
                    world_coords = [150,125,0+b_stack*36]
                    b_stack += 1
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    self.place_at_position_task345(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag= 0)
                elif block_info[5] == "small" and block_info[4] == c_order[s_stack]: 
                    world_coords = [block_info[0],block_info[1],block_info[2]]
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 1)
                    world_coords = [-150,-125,0+s_stack*26]
                    s_stack += 1
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                    self.place_at_position_task345(wrist_pos, gamma=gamma,block_theta=-np.pi/2, block_flag= 1)
                if s_stack == 6 or b_stack == 6:
                    complete = True
                    break
        self.rxarm.sleep()
        self.next_state = "idle"

    def task5(self):
        _ = self.camera.blockDetector()
        block_locations = self.camera.block_loc_xyz
        self.initialize_rxarm()
        no_blocks = 0
        for block in block_locations:
            block_info = block[1]
            if np.abs(block_info[0]) > 50:
                world_coords = [block_info[0],block_info[1],block_info[2]]
                wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                self.pick_at_position(wrist_pos, gamma=gamma, block_theta=block_info[3]/180*np.pi, block_flag = 0)
                world_coords = [0,175,0+no_blocks*35]
                 
                no_blocks += 1
                if no_blocks > 5:
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords, force_straight_wrist=1)
                else:
                    wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
                #TODO: need some sort of waypoint or wont stack very high, set block_flag to 2 for waypoint addition
                self.place_at_position_task345(wrist_pos, gamma=gamma, block_theta=np.pi/2, block_flag= 2)

    def unstack(self):
        for i in range(4):
            change = False
            self.rxarm.sleep()
            rospy.sleep(2)
            _ = self.camera.blockDetector()
            block_locations = self.camera.block_loc_xyz
            
            for block in block_locations:
                block_info = block[1]
                if block_info[2] < 200:
                    if block_info[5] == "big" and block_info[2] > 40:
                        world_coords = [block_info[0],block_info[1],block_info[2]+25]
                        self.move_and_knock(world_coords,0)
                        change = True
                    if block_info[5] == "small" and block_info[2] > 40:
                        world_coords = [block_info[0],block_info[1],block_info[2]+15]
                        self.move_and_knock(world_coords,1)
                        change = True
            if not change:
                break

    def move_and_knock(self,world_coords,block_flag):
        #attack from right to left
        wait_t = 1.5
        world_coords[0] += 75
        wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
        wrist_pos_pick = self.rxarm.get_wrist_pose_pick(wrist_pos, block_flag=block_flag)
        all_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_pick, gamma, 0)
        self.rxarm.set_moving_time(1.5)
        self.rxarm.set_positions(all_joint_angles[0])
        rospy.sleep(1.5)
        world_coords[0] -= 150
        wrist_pos, gamma = self.rxarm.get_wrist_pose(world_coords)
        wrist_pos_pick = self.rxarm.get_wrist_pose_pick(wrist_pos, block_flag=block_flag)
        all_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_pick, gamma, 0)
        self.rxarm.set_positions(all_joint_angles[0])
        rospy.sleep(1.5)
        self.rxarm.set_moving_time(1.5)
        self.rxarm.set_positions([0,0,0,0,0])
        rospy.sleep(1.5)
        

    def pick_at_position(self, wrist_pos, block_theta=0, gamma=-np.pi/2, block_flag=0):
        """ ASSUMES WE INPUT WRIST POSITION, !!!not!!! orig block position
        Default block flag is 0 for big block, 1 = small block
        """
        try:
            success = False
            
            wrist_pos_pick_raised = self.rxarm.get_wrist_pose_pick_raised(wrist_pos, gamma)
            all_joint_angles_raised = self.rxarm.move_to_ee_pose(wrist_pos_pick_raised, gamma, block_theta)
            wrist_pos_pick = self.rxarm.get_wrist_pose_pick(wrist_pos, block_flag=block_flag)
            all_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_pick, gamma, block_theta)
            wrist_pos_safe = self.rxarm.get_wrist_pose_place_safe(wrist_pos)
            safe_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_safe, gamma, block_theta)
            self.rxarm.set_moving_time(1.0)
            self.rxarm.open_gripper()
            rospy.sleep(1.0 * 0.5)
            
            self.rxarm.set_moving_time(2.0)
            self.rxarm.set_positions([all_joint_angles_raised[0][0], 0, 0, 0, 0])
            rospy.sleep(2.0)
            
            self.rxarm.set_moving_time(1.0)
            self.rxarm.set_positions(all_joint_angles_raised[0])
            rospy.sleep(2.0* 0.5)
            self.rxarm.set_positions(all_joint_angles[0])  # only move downwards
            rospy.sleep(2.0* 0.5)
            
            self.rxarm.set_moving_time(1.0)
            self.rxarm.close_gripper()
            rospy.sleep(0.5)
            
            # Move to a safe distance away from block that was just placed
            self.rxarm.set_positions(safe_joint_angles[0])
            rospy.sleep(1.0)
            self.rxarm.set_moving_time(2.0)
        except:
            return success
        else:
            success = True
            return success
        
    def place_at_position(self, wrist_pos, block_theta=0, gamma=-np.pi/2, block_flag=0):
        """ ASSUMES WE INPUT WRIST POSITION, !!!not!!! orig block position
        """
        try:
            success = False
            
            wrist_pos_place_raised = self.rxarm.get_wrist_pose_place_raised(wrist_pos)
            all_joint_angles_raised = self.rxarm.move_to_ee_pose(wrist_pos_place_raised, gamma, block_theta)
            
            wrist_pos_place = self.rxarm.get_wrist_pose_place(wrist_pos, block_flag=block_flag)
            all_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_place, gamma, block_theta)
            
            wrist_pos_safe = self.rxarm.get_wrist_pose_place_safe(wrist_pos)
            safe_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_safe, gamma, block_theta)
            
            self.rxarm.set_moving_time(2.0)
            self.rxarm.set_positions([all_joint_angles_raised[0][0], 0, 0, 0, 0])
            rospy.sleep(2.0)
            self.rxarm.set_moving_time(1.0)
            self.rxarm.set_positions(all_joint_angles_raised[0])
            rospy.sleep(2.0 * 0.5)
            self.rxarm.set_moving_time(1.0)
            self.rxarm.set_positions(all_joint_angles[0])
            rospy.sleep(1.0)
            self.rxarm.open_gripper()
            rospy.sleep(0.5)
            
            # Move to a safe distance away from block that was just placed
            self.rxarm.set_positions(safe_joint_angles[0])
            rospy.sleep(1.0)
            self.rxarm.set_moving_time(2.0)
        except:
            return success
        else:
            success = True
            return success
        
    def place_at_position_task345(self, wrist_pos, block_theta=0, gamma=-np.pi/2, block_flag=0):
        """ ASSUMES WE INPUT WRIST POSITION, !!!not!!! orig block position
        """
        try:
            success = False
            wrist_pos_place_raised = self.rxarm.get_wrist_pose_place_raised(wrist_pos)
            all_joint_angles_raised = self.rxarm.move_to_ee_pose(wrist_pos_place_raised, gamma, block_theta)
            
            wrist_pos_place = self.rxarm.get_wrist_pose_place(wrist_pos, block_flag=block_flag)
            all_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_place, gamma, block_theta)
            
            wrist_pos_safe = self.rxarm.get_wrist_pose_place_safe(wrist_pos)
            safe_joint_angles = self.rxarm.move_to_ee_pose(wrist_pos_safe, gamma, block_theta)
            
            if block_flag == 1:
                # small block for task 4
                self.rxarm.set_positions([np.pi/2,-0.3,0.3,0,0])
            elif block_flag == 2:
                # waypoint for task 5
                self.rxarm.set_positions([0,-0.3,0.3,0,0])
            else:
                # big block for task 4
                self.rxarm.set_positions([-np.pi/2,-0.3,0.3,0,0])
            rospy.sleep(2.0)
            self.rxarm.set_moving_time(1.0)
            self.rxarm.set_positions(all_joint_angles_raised[0])
            rospy.sleep(1.0)
            
            self.rxarm.set_positions(all_joint_angles[0])
            rospy.sleep(1.0)
            self.rxarm.open_gripper()
            rospy.sleep(0.5)
            
            # Move to a safe distance away from block that was just placed
            self.rxarm.set_positions(safe_joint_angles[0])
            rospy.sleep(1.0)
            self.rxarm.set_moving_time(2.0)
        except:
            return success
        else:
            success = True
            return success


class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)