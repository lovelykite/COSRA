import time

import numpy as np

from Trajectory import Trajectory
from dynamixel import dynamixel

# dxls = dynamixel()
dxl_ids = [0, 1, 2, 3]  # ID setting

# for i in range(3):
#     dxls.enable_torque(dxl_ids[i], True)  # torque enable


def homePosition():
    homePos = [1623, 1482, 1719, 2160]
    dxls.set_pos_sync(dxl_ids, homePos)
    # time.sleep(1)
    # print(dxls.get_pos_sync(dxl_ids))


def pickNplace(start_idx, goal_idx):
    t = 0.0
    tf = 3.0

    gripAngle = 1800  # 그리퍼가 물체 잡기 위해 필요한 각도 (0~4095)
    ungripAngle = 2160

    # home
    home = [1623, 1482, 1719]

    # 1st
    upper_1st = [2080, 1845, 2235]  # upper
    lower_1st = [2094, 1840, 2078]  # lower

    # 2nd
    upper_2nd = [2147, 1520, 1740]
    lower_2nd = [2150, 1530, 1725]

    # 3rd
    upper_3rd = [1984, 1730, 2180]
    lower_3rd = [1984, 1765, 2126]

    # 4th
    upper_4th = [1984, 1522, 1734]
    lower_4th = [1984, 1527, 1692]

    # 5th
    upper_5th = [1839, 1770, 2175]
    lower_5th = [1839, 1809, 2175]

    # 6th
    upper_6th = [1816, 1542, 1773]
    lower_6th = [1803, 1562, 1773]

    # cart_pick = np.array(start)  # pick 해야 하는 물체의 cartesian
    # print(cart_pick)
    # cart_via1 = np.array(start)
    # cart_via1[2] += 0.04  # print(cart_via1)
    # cart_place = np.array(goal)  # place 해야 하는 곳의 cartesian
    # print(cart_place)
    # cart_via2 = np.array(goal)
    # print(cart_via2)

    start_upper = np.zeros(3)
    start_lower = np.zeros(3)
    goal_upper = np.zeros(3)
    goal_lower = np.zeros(3)

    upper = [upper_1st, upper_2nd, upper_3rd, upper_4th, upper_5th, upper_6th]
    lower = [lower_1st, lower_2nd, lower_3rd, lower_4th, lower_5th, lower_6th]

    for i in range(6):
        if start_idx == i:
            start_upper = upper[i - 1]
            start_lower = lower[i - 1]
        if goal_idx == i:
            goal_upper = upper[i - 1]
            goal_lower = lower[i - 1]

    # if start_idx == 1:
    #     start_upper = upper_1st
    #     start_lower = lower_1st
    # elif start_idx == 2:
    #     start_upper = upper_2nd
    #     start_lower = lower_2nd
    # elif start_idx == 3:
    #     start_upper = upper_3rd
    #     start_lower = lower_3rd
    # elif start_idx == 4:
    #     start_upper = upper_4th
    #     start_lower = lower_4th
    # elif start_idx == 5:
    #     start_upper = upper_5th
    #     start_lower = lower_5th
    # else:
    #     start_upper = upper_6th
    #     start_lower = lower_6th
    #
    # if goal_idx == 1:
    #     goal_upper = upper_1st
    #     goal_lower = lower_1st
    # elif goal_idx == 2:
    #     goal_upper = upper_2nd
    #     goal_lower = lower_2nd
    # elif goal_idx == 3:
    #     goal_upper = upper_3rd
    #     goal_lower = lower_3rd
    # elif goal_idx == 4:
    #     goal_upper = upper_4th
    #     goal_lower = lower_4th
    # elif goal_idx == 5:
    #     goal_upper = upper_5th
    #     goal_lower = lower_5th
    # else:
    #     goal_upper = upper_6th
    #     goal_lower = lower_6th

    """ Cartesian Trajectories """
    # Trajectory 1: Move to Start point horizontally
    traj_s1, t1 = Trajectory.LSPB(q0=home, qf=start_upper, tf=tf, tb=tf / 3)

    # Trajectory 2: Go down for Gripping
    traj_s2, t2 = Trajectory.LSPB(q0=start_upper, qf=start_lower, tf=tf, tb=tf / 3)

    # Trajectory 3: Pick up
    traj_s3, t3 = Trajectory.LSPB(q0=start_lower, qf=start_upper, tf=tf, tb=tf / 3)

    # Trajectory 4: Move to Goal point horizontally
    traj_s4, t4 = Trajectory.LSPB(q0=start_upper, qf=goal_upper, tf=tf, tb=tf / 3)

    # Trajectory 5: Place
    traj_s5, t5 = Trajectory.LSPB(q0=goal_upper, qf=goal_lower, tf=tf, tb=tf / 3)

    # Trajectory 6: Return
    traj_s6, t6 = Trajectory.LSPB(q0=goal_lower, qf=goal_upper, tf=tf, tb=tf / 3)

    # Trajectory 7: Home coming
    traj_s7, t7 = Trajectory.LSPB(q0=goal_upper, qf=home, tf=tf, tb=tf / 3)

    """ Joint Trajectories """

    move_joint(traj_s1, t1, "1", start_lower, goal_lower, gripAngle, ungripAngle)

    move_joint(traj_s2, t2, "2", start_lower, goal_lower, gripAngle, ungripAngle, 1)

    move_joint(traj_s3, t3, "3", start_lower, goal_lower, gripAngle, ungripAngle)

    move_joint(traj_s4, t4, "4", start_lower, goal_lower, gripAngle, ungripAngle)

    move_joint(traj_s5, t5, "5", start_lower, goal_lower, gripAngle, ungripAngle, 2)

    move_joint(traj_s6, t6, "6", start_lower, goal_lower, gripAngle, ungripAngle)

    move_joint(traj_s7, t7, "7", start_lower, goal_lower, gripAngle, ungripAngle)


def move_joint(traj_s, t, pos_num, start_lower, goal_lower, gripAngle, ungripAngle, grip=0):
    print("move position " + pos_num)
    for i in range(len(t)):
        start_time = time.time()
        for j in range(3):
            # print("traj_s(0): ", traj_s[i][0], "traj_s(1): ", traj_s[i][1], "traj_s(2): ", traj_s[i][2])
            dxls.set_pos(dxl_ids[j], traj_s[i][j])
        print("time: ", time.time() - start_time, "traj_s :", traj_s[i])

        if grip == 1:
            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian
            print("current cartesian: ", cart_cur)
            print("pick cartesian: ", start_lower)
            ''' Grip '''
            if np.linalg.norm(cart_cur - start_lower) < 10e-4:
                dxls.set_pos(dxl_ids[3], gripAngle)

        elif grip == 2:
            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian
            print("current cartesian: ", cart_cur)
            print("place cartesian: ", goal_lower)
            ''' Place '''
            if np.linalg.norm(cart_cur - goal_lower) < 10e-4:
                dxls.set_pos(dxl_ids[3], ungripAngle)
        else:
            pass


# cart_start = [0.18, 0.15, 0.02]
# cart_goal = [0.18, -0.15, 0.02]
start_idx = 1
goal_idx = 2

HomeFlag = False
pickNplace(start_idx, goal_idx)
HomeFlag = True
time.sleep(2)
# homePosition()

#
# def dxlPos2rad(pos=[]):
#     # dxl position: 0 ~ 4095 (0 degree at 2048)
#     joints = 2 * np.array(pos) * np.pi / 4096
#     return joints
#
#
# def rad2dxlPos(joints=[]):
#     pos = 4096 * np.array(joints) / (2 * np.pi)
#     return pos
