"""
/*** Pick & Place Algorithm ***/
1. Trajectories:

1.1. Cartesian coordinates
    - Obtained from

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


def pickNplace(start_idx, goal_idx, cart=[]):
    gripAngle   = 1670  # Gripper angle for Grasping  (0 ~ 4095)
    ungripAngle = 2250  # Gripper angle for Releasing (0 ~ 4095)

    ''' Cartesian Space: Home, Start, Goal, Via points' Cartesian coordinates [x, y, z] '''
    # home = [1460, 1500, 1590]       # home position Joint angle [pulse]
    cart_home = cart  # home position Cartesian coordinates [x, y, z]
    # print("cart_home: ", cart_home)

    # 1st
    upper_1st = np.array([0.313, 0.025, 0.168])  # upper
    lower_1st = np.array([0.282, 0.022, 0.218])  # lower

    # 2nd
    upper_2nd = np.array([0.288, 0.032, 0.137])
    lower_2nd = np.array([0.231, 0.037, 0.155])

    # 3rd
    upper_3rd = np.array([0.310, -0.034, 0.179])
    lower_3rd = np.array([0.290, -0.031, 0.201])

    # 4th
    upper_4th = np.array([0.290, -0.051, 0.110])
    lower_4th = np.array([0.240, -0.037, 0.154])

    # 5th
    upper_5th = np.array([0.300, -0.089, 0.180])
    lower_5th = np.array([0.280, -0.084, 0.208])

    # 6th
    upper_6th = np.array([0.260, -0.122, 0.121])
    lower_6th = np.array([0.220, -0.100, 0.182])

    upper = np.array([upper_1st, upper_2nd, upper_3rd, upper_4th, upper_5th, upper_6th])
    lower = np.array([lower_1st, lower_2nd, lower_3rd, lower_4th, lower_5th, lower_6th])

    start_upper = upper[start_idx - 1][:]
    start_lower = lower[start_idx - 1][:]
    goal_upper  = upper[goal_idx - 1][:]
    goal_lower  = lower[goal_idx - 1][:]

    ''' Cartesian Trajectories '''
    tf = 3.0  # time interval for Interpolation
    traj_s1, t1 = Trajectory.LSPB(q0=cart_home,   qf=start_upper, tf=tf, tb=tf / 3)  # Trajectory 1: Home to Start Upper
    traj_s2, t2 = Trajectory.LSPB(q0=start_upper, qf=start_lower, tf=tf, tb=tf / 3)  # Trajectory 2: Move to Start Lower
    traj_s3, t3 = Trajectory.LSPB(q0=start_lower, qf=start_upper, tf=tf, tb=tf / 3)  # Trajectory 3: Move to Start Upper
    traj_s4, t4 = Trajectory.LSPB(q0=start_upper, qf=goal_upper,  tf=tf, tb=tf / 3)  # Trajectory 4: Move to Goal Upper
    traj_s5, t5 = Trajectory.LSPB(q0=goal_upper,  qf=goal_lower,  tf=tf, tb=tf / 3)  # Trajectory 5: Move to Goal Lower
    traj_s6, t6 = Trajectory.LSPB(q0=goal_lower,  qf=goal_upper,  tf=tf, tb=tf / 3)  # Trajectory 6: Move to Goal Upper
    traj_s7, t7 = Trajectory.LSPB(q0=goal_upper,  qf=cart_home,   tf=tf, tb=tf / 3)  # Trajectory 7: Goal Upper to Home

    ''' Joint Trajectories: Dynamixel Operation '''
    move_joint(traj_s1, t1, "1")
    move_joint(traj_s2, t2, "2")

    # Pick
    dxls.set_pos(dxl_ids[3], gripAngle)
    time.sleep(1)

    move_joint(traj_s3, t3, "3")
    move_joint(traj_s4, t4, "4")
    move_joint(traj_s5, t5, "5")

    # Place
    dxls.set_pos(dxl_ids[3], ungripAngle)
    time.sleep(1)

    move_joint(traj_s6, t6, "6")
    move_joint(traj_s7, t7, "7")


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
# dxl position: 0 ~ 4095 (0 degree at 2048)
def dxlPos2rad(pos=[]):
    joints = 2 * np.array(pos) * np.pi / 4095
    return joints


def rad2dxlPos(joints=[]):
    pos = 4095 * np.array(joints) / (2 * np.pi)
    return pos


# def initPosition():
#     initJoint = dxls.get_pos_sync(dxl_ids)
#     return initJoint


def callback_joint(data):
    global start_p
    global goal_p
    start_p = np.array(data.data)[0]
    goal_p = np.array(data.data)[1]


def listen_target():
    rospy.init_node('control', anonymous=True)
    rospy.Subscriber("send_goal", Int32MultiArray, callback_joint)


if __name__ == '__main__':
    ''' Dynamixel Setting '''
    dxls = dynamixel()
    dxl_ids = [0, 1, 2, 3]  # ID setting

    for i in range(4):
        dxls.enable_torque(dxl_ids[i], True)  # Torque Enable

    ''' Initial Position Setting '''
    # initPosition()
    # dxls.set_pos_sync(dxl_ids, initPosition())

    initJoint = dxls.get_pos_sync(dxl_ids)
    dxls.set_pos_sync(dxl_ids, initJoint)

    # cart_init = cosraKinematics.fk(dxlPos2rad(initPosition()[:3]))[:3, -1]
    # print("Initial [x, y, z] : ", np.round(cart_init, 3))

    ''' Move from Initial Position to Home Position '''
    homePosition = [1460, 1400, 1750, 2250]  # pulse value for each joints

    tf0 = 3.  # time interval for Interpolation
    traj_s0, t0 = Trajectory.LSPB(q0=initPosition(), qf=homePosition, tf=tf0, tb=tf0 / 3)

    for i in range(len(t0)):
        # start_time = time.time()
        q_home = dxls.get_pos_sync(dxl_ids)[:3]                     # pulse value for each joints in trajectories
        cart_home = cosraKinematics.fk(dxlPos2rad(q_home))[:3, -1]  # home cartesian coordinates
        # cart_home = cosraKinematics.fk(dxlPos2rad(dxls.get_pos_sync(dxl_ids)[:3]))[:3, -1]

        for j in range(4):
            dxls.set_pos(dxl_ids[j], int(traj_s0[i][j]))
        # print("time: ", round(time.time() - start_time, 4), "traj_s :", np.round(traj_s0[i], 4))
    print("q_home: ", q_home)
    print("Home [x, y, z] : ", np.round(cart_home, 3))

    start_idx = 6
    goal_idx = 1

    ''' Pick & Place '''
    try:
        while True:
            # listen_target()
            pickNplace(start_idx, goal_idx, cart=cart_home)
    except KeyboardInterrupt:
        pass
