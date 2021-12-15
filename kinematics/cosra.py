import time

import numpy as np
from Trajectory import Trajectory
from dynamixel import dynamixel

import rospy
from std_msgs.msg import Int32, Int32MultiArray

dxls = dynamixel()
dxl_ids = [0, 1, 2, 3]  # ID setting
dxls = dynamixel()

for i in range(3):
    dxls.enable_torque(dxl_ids[i], True)  # torque enable


def homePosition():
    homePos = [1623, 1482, 1719, 2160]
    dxls.set_pos_sync(dxl_ids, homePos)
    # time.sleep(1)
    # print(dxls.get_pos_sync(dxl_ids))

gripAngle = 1800  # 그리퍼가 물체 잡기 위해 필요한 각도 (0~4095)
ungripAngle = 2160


def pickNplace(start_idx, goal_idx):
    t = 0.0
    tf = 3.0

    gripAngle = 1800  # 그리퍼가 물체 잡기 위해 필요한 각도 (0~4095)
    ungripAngle = 2160

    # home
    home = [1558, 1300, 1948]

    # 1st
    upper_1st = [2092, 1737, 1345]  # upper
    lower_1st = [2092, 1737, 2145]  # lower

    # 2nd
    upper_2nd = [2165, 1373, 1764]
    lower_2nd = [2105, 1361, 1694]

    # 3rd
    upper_3rd = [1962, 1562, 2188]
    lower_3rd = [1962, 1581, 2102]

    # 4th
    upper_4th = [1962, 1402, 1765]
    lower_4th = [1962, 1402, 1663]

    # 5th
    upper_5th = [1838, 1581, 2283]
    lower_5th = [1838, 1657, 2235]

    # 6th
    upper_6th = [1792, 1186, 1880]
    lower_6th = [1792, 1424, 1682]

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

    upper = np.array([upper_1st, upper_2nd, upper_3rd, upper_4th, upper_5th, upper_6th])
    lower = np.array([lower_1st, lower_2nd, lower_3rd, lower_4th, lower_5th, lower_6th])

    
    start_upper = upper[start_idx - 1][:]
    start_lower = lower[start_idx - 1][:]

    goal_upper = upper[goal_idx - 1][:]
    goal_lower = lower[goal_idx - 1][:]

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

    move_joint(traj_s1, t1, "1")

    move_joint(traj_s2, t2, "2")

    for i in range (10):
        dxls.set_pos(dxl_ids[3], int(gripAngle/10*i))
        time.sleep(0.1)

    move_joint(traj_s3, t3, "3")

    move_joint(traj_s4, t4, "4")

    move_joint(traj_s5, t5, "5")

    for i in range (10):
        dxls.set_pos(dxl_ids[3], int(ungripAngle/10*i))
        time.sleep(0.1)

    move_joint(traj_s6, t6, "6")

    move_joint(traj_s7, t7, "7")


def move_joint(traj_s, t, pos_num):
    print("move position " + pos_num)
    for i in range(len(t)):
        start_time = time.time()
        for j in range(3):
            # print("traj_s(0): ", traj_s[i][0], "traj_s(1): ", traj_s[i][1], "traj_s(2): ", traj_s[i][2])
            dxls.set_pos(dxl_ids[j], int(traj_s[i][j]))
        time.sleep(0.06)
        print("time: ", time.time() - start_time, "traj_s :", traj_s[i])


# cart_start = [0.18, 0.15, 0.02]
# cart_goal = [0.18, -0.15, 0.02]
start_idx = 1
goal_idx = 2

start_p = None
goal_p = None

def callback_joint(data):
    global start_p
    global goal_p
    start_p = np.array(data.data)[0]
    goal_p = np.array(data.data)[1]

def listen_target():
    rospy.init_node('control', anonymous=True)
    rospy.Subscriber("send_goal", Int32MultiArray, callback_joint)

if __name__ == '__main__':
    try:
        while True:
            # listen_target()
            pickNplace(start_idx, goal_idx)
            # for i in range(4):
            #     print(dxls.get_pos(i))
            # pickNplace(start_p, goal_p)
            # for i in range (10):
            #     dxls.set_pos(dxl_ids[3], int(gripAngle/10*i))
            #     time.sleep(0.1)
            # for i in range (10):
            #     dxls.set_pos(dxl_ids[3], int(ungripAngle/10*i))
            #     time.sleep(0.1)
            
    except KeyboardInterrupt:
        pass

    
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