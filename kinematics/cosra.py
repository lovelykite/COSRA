"""
### Pick & Place Algorithm ###
# Copyright
update: 2021.12.19
made by: Yeon Kang, and Jeong-woo Hong

# Description
1. Trajectories
    - Traejectories are generated based on Linear Segment with Parabolic Blend (LSPB) method
    - Time step (t_step = 0.07) for LSPB is set based on calculation time of 'move_joint' function
    
1.1. Cartesian Space: End-effector Cartesian Trajectories
    - Cartesian coordinates of end-effector is predefined based on each color section which is indexed as number 1 to 6
        - Start Upper Position, Start Lower Position, Goal Upper Position, Goal Lower Position
    - Initial Position, and Home Position are set so as not to disturb RGB Camera visual range
    
1.2. Joint Space: Required Joints Position (angle) Trajectories
    - Based on cartesian trajectories, required joint position trajectories are obtained by Inverse Kinematics
   
   
2. Colored Box Sorting Process
     - With Image Processing, information about which box should be picked up (start_p), and where should it be placed (goal_p)
     is trasmitted as arguments to 'pickNplace' function
     - If COSRA accomplished sorting process, it shows ceremony
    
"""

import time

import numpy as np
from numpy import round
from scipy.optimize import fsolve

from Trajectory import Trajectory
from cosraKinematics import cosraKinematics
from dynamixel import dynamixel

import rospy
from std_msgs.msg import Int32, Int32MultiArray
import threading


def pickNplace(start_idx, goal_idx, cart=[]):
    gripAngle   = 1660  # Gripper angle for Grasping  (0 ~ 4095)
    ungripAngle = 2300  # Gripper angle for Releasing (0 ~ 4095)

    ''' Cartesian Space: Home, Start, Goal, Via points' Cartesian coordinates [x, y, z] '''
    cart_home = cart  # home position Cartesian coordinates [x, y, z]
    # print("cart_home: ", cart_home)

    # 1st
    upper_1st = np.array([0.313, 0.035, 0.170])  # upper
    lower_1st = np.array([0.282, 0.032, 0.220])  # lower

    # 2nd
    upper_2nd = np.array([0.298, 0.042, 0.127])
    lower_2nd = np.array([0.241, 0.047, 0.165])

    # 3rd
    upper_3rd = np.array([0.310, -0.034, 0.179])
    lower_3rd = np.array([0.290, -0.031, 0.201])

    # 4th
    upper_4th = np.array([0.290, -0.051, 0.110])
    lower_4th = np.array([0.240, -0.037, 0.154])

    # 5th
    upper_5th = np.array([0.300, -0.110, 0.180])
    lower_5th = np.array([0.280, -0.105, 0.208])

    # 6th
    upper_6th = np.array([0.270, -0.120, 0.101])
    lower_6th = np.array([0.200, -0.090, 0.182])

    # Ceremony Positions
    upper_7th = (upper_1st + upper_2nd) / 2     # 7th
    upper_8th = (upper_5th + upper_6th) / 2     # 8th

    # Center Position
    center = (upper_3rd + upper_4th) / 2

    upper = np.array([upper_1st, upper_2nd, upper_3rd, upper_4th, upper_5th, upper_6th])
    lower = np.array([lower_1st, lower_2nd, lower_3rd, lower_4th, lower_5th, lower_6th])

    tf = 3.0  # time interval for Interpolation

    if start_idx == 7 and goal_idx == 8:    # Checking whether colored box sorting operation is accomplished
        ''' Ceremony '''
        traj_sc1, tc1 = Trajectory.LSPB(q0=cart_home, qf=upper_7th, tf=tf, tb=tf / 3)
        traj_sc2, tc2 = Trajectory.LSPB(q0=upper_7th, qf=upper_8th, tf=tf, tb=tf / 3)
        traj_sc3, tc3 = Trajectory.LSPB(q0=upper_8th, qf=upper_7th, tf=tf, tb=tf / 3)
        traj_sc4, tc4 = Trajectory.LSPB(q0=upper_7th, qf=upper_8th, tf=tf, tb=tf / 3)
        traj_sc5, tc5 = Trajectory.LSPB(q0=upper_8th, qf=cart_home, tf=tf, tb=tf / 3)

        move_joint(traj_sc1, tc1, "1")
        move_joint(traj_sc2, tc2, "2")
        move_joint(traj_sc3, tc3, "3")
        move_joint(traj_sc4, tc4, "4")
        move_joint(traj_sc5, tc5, "5")

        dxls.close_port()
        return
    else:   # Colored box sorting operation
        start_upper = upper[start_idx - 1][:]
        start_lower = lower[start_idx - 1][:]
        goal_upper  = upper[goal_idx - 1][:]
        goal_lower  = lower[goal_idx - 1][:]

        ''' Cartesian Trajectories '''
        traj_s1, t1 = Trajectory.LSPB(q0=cart_home, qf=center, tf=tf, tb=tf / 3)  # Path 1: Home to Center
        traj_s2, t2 = Trajectory.LSPB(q0=center, qf=start_upper, tf=tf, tb=tf / 3)  # Path 2: Center to Start upper
        traj_s3, t3 = Trajectory.LSPB(q0=start_upper, qf=start_lower, tf=tf, tb=tf / 3)  # Path 3: Start upper to Start lower
        traj_s4, t4 = Trajectory.LSPB(q0=start_lower, qf=start_upper, tf=tf, tb=tf / 3)  # Path 4: Start lower to Start upper
        traj_s5, t5 = Trajectory.LSPB(q0=start_upper, qf=center, tf=tf, tb=tf / 3)  # Path 5: Start upper to Center
        traj_s6, t6 = Trajectory.LSPB(q0=center, qf=goal_upper, tf=tf, tb=tf / 3)  # Path 6: Cetner to Goal upper
        traj_s7, t7 = Trajectory.LSPB(q0=goal_upper, qf=goal_lower, tf=tf, tb=tf / 3)  # Path 7: Goal upper to Goal lower
        traj_s8, t8 = Trajectory.LSPB(q0=goal_lower, qf=goal_upper, tf=tf, tb=tf / 3)  # Path 8: Goal lower to Goal upper
        traj_s9, t9 = Trajectory.LSPB(q0=goal_upper, qf=center, tf=tf, tb=tf / 3)  # Path 9: Goal upper to Center
        traj_s10, t10 = Trajectory.LSPB(q0=center, qf=cart_home, tf=tf, tb=tf / 3)  # Path 10: Center to Home

        ''' Joint Trajectories: Dynamixel Operation '''
        move_joint(traj_s1, t1, "1")
        move_joint(traj_s2, t2, "2")
        move_joint(traj_s3, t3, "3")

        # Pick
        dxls.set_pos(dxl_ids[3], gripAngle)
        time.sleep(1)

        move_joint(traj_s4, t4, "4")
        move_joint(traj_s5, t5, "5")
        move_joint(traj_s6, t6, "6")
        move_joint(traj_s7, t7, "7")

        # Place
        dxls.set_pos(dxl_ids[3], ungripAngle)
        time.sleep(1)

        move_joint(traj_s8, t8, "8")
        move_joint(traj_s9, t9, "9")
        move_joint(traj_s10, t10, "10")

''' Dynamixel operation based on Inverse Kinematics '''
def move_joint(traj_s, t, pos_num):
    print("move position " + pos_num)

    for i in range(len(t)):
        start_time = time.time()  # time stamp for checking Calculation time

        ''' Initial value for Inverse Kinematics '''
        q_cur = dxls.get_pos_sync(dxl_ids)[:3]  # get pulse angle from dynamixels
        q_rad = dxlPos2rad(q_cur)               # convert pulse angle to radian

        ''' Inverse Kinematics '''
        # q_rad_act: actual joint value [rad] including initial position compensation
        q_rad_act = fsolve(cosraKinematics.ik, x0=q_rad, args=tuple(traj_s[i]))         # analytical sol
        # q_rad_act = fsolve(cosraKinematics.ik_geo, x0=q_rad, args=tuple(traj_s[i]))     # geometrical sol

        cart_cur = cosraKinematics.fk(q_rad)[:3, -1]  # theoretical cartesian coordinates
        # cart_cur = cosraKinematics.fk(q_rad_act)[:3, -1]  # actual cartesian coordinates
        # print("cart_cur: ", cart_cur)

        dxl_pos = np.array(rad2dxlPos(q_rad_act))  # actual joint value [pulse]
        dxl_pos = dxl_pos.astype(int)              # type conversion

        for j in range(3):
            # print("traj_s(0): ", traj_s[i][0], "traj_s(1): ", traj_s[i][1], "traj_s(2): ", traj_s[i][2])
            dxls.set_pos(dxl_ids[j], dxl_pos[j])
        # time.sleep(0.06)
        print("Elapsed time: ", round(time.time() - start_time, 4), "trajectory: ", np.round(traj_s[i], 3),
              "Actual dxl_pulse", dxl_pos, " Actual radian: ", q_rad_act)
    print("Current [x, y, z] : ", np.round(cart_cur, 3), '\n')
    time.sleep(0.5)


''' Angle value Conversion b/w Pulse value & Radian '''
# dxl position: 0 ~ 4095 (pi rad at 2048)
def dxlPos2rad(pos=[]):
    joints = 2 * np.array(pos) * np.pi / 4095
    return joints


def rad2dxlPos(joints=[]):
    pos = 4095 * np.array(joints) / (2 * np.pi)
    return pos


start_p = None
goal_p = None

def callback_joint(data):
    global start_p
    global goal_p
    start_p = np.array(data.data)[0]
    goal_p = np.array(data.data)[1]


def listen_target():
    rospy.Subscriber("send_goal", Int32MultiArray, callback_joint)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)

    listener = threading.Thread(target = listen_target)
    listener.start()
    print(start_p, ", ", goal_p)

    while not rospy.is_shutdown():

        ''' Dynamixel Setting '''
        dxls = dynamixel()
        dxl_ids = [0, 1, 2, 3]  # ID setting

        for i in range(4):
            dxls.enable_torque(dxl_ids[i], True)  # Torque Enable

        ''' Initial Position Setting '''
        initJoint = dxls.get_pos_sync(dxl_ids)
        dxls.set_pos_sync(dxl_ids, initJoint)

        # cart_init = cosraKinematics.fk(dxlPos2rad(initJoint[:3]))[:3, -1]
        # print("Initial [x, y, z] : ", np.round(cart_init, 3))

        ''' Move from Initial Position to Home Position '''
        homePosition = [1460, 1400, 1750, 2250]  # pulse value for each joints

        tf0 = 3.  # time interval for Interpolation
        traj_s0, t0 = Trajectory.LSPB(q0=initJoint, qf=homePosition, tf=tf0, tb=tf0 / 3)

        for i in range(len(t0)):
            # start_time = time.time()
            q_home = dxls.get_pos_sync(dxl_ids)[:3]                     # pulse value for each joints in trajectories
            cart_home = cosraKinematics.fk(dxlPos2rad(q_home))[:3, -1]  # home cartesian coordinates

            for j in range(4):
                dxls.set_pos(dxl_ids[j], int(traj_s0[i][j]))
            # print("time: ", round(time.time() - start_time, 4), "traj_s0:", np.round(traj_s0[i], 4))
        print("q_home: ", q_home)
        print("Home [x, y, z] : ", np.round(cart_home, 3))

        print(start_p, ", ", goal_p)
        
        ''' Pick & Place '''
        pickNplace(start_p, goal_p, cart=cart_home)

    rospy.spin()

    # start_idx = 6
    # goal_idx = 1

    ''' Pick & Place '''
    
    # try:
    #     while True:
    #         listen_target()
    #         print(start_p, ", ", goal_p)
    #         pickNplace(start_p, goal_p, cart=cart_home)
    #         # pickNplace(start_idx, goal_idx, cart=cart_home)
    # except KeyboardInterrupt:
    #     pass
