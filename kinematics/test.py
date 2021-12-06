import time

import numpy as np
from numpy import sin, cos
from scipy.optimize import fsolve

from Trajectory import Trajectory
from cosraKinematics import cosraKinematics
from dynamixel import dynamixel

dxls = dynamixel()
dxl_ids = [0, 1, 2, 3]  # ID setting

for i in range(3):
    dxls.enable_torque(dxl_ids[i], True)  # torque enable

# link length a(i-1)
l1 = 0.095  # J1 ~ J2
l2 = 0.150  # J2 ~ J3
l3 = 0.150  # J3 ~ ee
lg = 0.112  # gripper size
lt = 0.050  # tip location
h  = 0.040  # base height

# joint angle qi
# q = [3.0987, 0.3068, 0.9203]
q = [0.5236, 0.5236, 0.5236]

def dhparam(joints):
    q1, q2, q3 = np.array(joints).T
    # dh parameters [alpha, a, d, theta]
    return np.array([[0, 0, 0, q1],
                     [np.pi / 2, l1, 0, q2],
                     [-np.pi, l2, 0, q3],
                     [0, l3, 0, 0]])


# print(dhparam(q))

def DH_transform(dhparams):  # stacks transforms of neighbor frame, following the modified DH convention
    Ts = [np.array([[np.cos(theta), -np.sin(theta), 0, a],
                    [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),
                     -np.sin(alpha) * d],
                    [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha),
                     np.cos(alpha) * d],
                    [0, 0, 0, 1]]) for [alpha, a, d, theta] in dhparams]
    return Ts


# print(DH_transform(dhparam(q)))

def fk(joints):
    Ts = DH_transform(dhparam(joints))  # Tb1, T12, T23, ...
    # Tbe = np.linalg.multi_dot(Ts)   # from base to end-effector
    Tbs = np.array(
        [np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, len(Ts) + 1)])  # Tb1, Tb2, Tb3, ...
    # Tbs[-1]: from base to end effector
    # return Ts, Tbs[-1]  # base to end effector
    return Tbs[-1]

print(fk(q))

# q_cur = [np.pi / 3, np.pi / 3, np.pi / 3]

### cartesian coordinates ~ joint angle
# px = l1 * cos(q1) + l2 * cos(q1) * cos(q2) + l3 * (sin(q2) * sin(q3) * cos(q1) + cos(q1) * cos(q2) * cos(q3))
# py = l1 * sin(q1) + l2 * sin(q1) * cos(q2) + l3 * (sin(q1) * sin(q2) * sin(q3) + sin(q1) * cos(q2) * cos(q3))
# pz = l2 * sin(q2) + l3 * (sin(q2) * cos(q3) - sin(q3) * cos(q2))

def ik(q, *cart_des):
    x, y, z = cart_des

    q1, q2, q3 = q
    F = np.empty((3))

    F[0] = l1 * cos(q1) + l2 * cos(q1) * cos(q2) + l3 * (sin(q2) * sin(q3) * cos(q1) + cos(q1) * cos(q2) * cos(q3)) - x
    F[1] = l1 * sin(q1) + l2 * sin(q1) * cos(q2) + l3 * (sin(q1) * sin(q2) * sin(q3) + sin(q1) * cos(q2) * cos(q3)) - y
    F[2] = l2 * sin(q2) + l3 * (sin(q2) * cos(q3) - sin(q3) * cos(q2)) - z
    # print(q)
    return F


# cart_des = (0.32468, 0.18745, 0.075)
# q_cur = [0.3, 0.3, 0.3]
# qk = fsolve(ik(cart_des), q_cur)
# q = fsolve(ik, x0=q_cur, args=cart_des)
# print(q)


def pickNplace(start=[], goal=[]):
    t = 0.0
    tf = 3.0
    gripAngle = 333  # 그리퍼가 물체 잡기 위해 필요한 각도 (0~4095)
    ungripAngle = 0

    # Via points
    q_home = dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))  # 각 id별 dxl position을 받아 rad으로 변환
    # print(q_home)
    cart_home = cosraKinematics.fk(joints=q_home)[:3, -1]  # 현재 ee cartesian
    # print(cart_home)

    cart_pick = np.array(start)  # pick 해야 하는 물체의 cartesian
    # print(cart_pick)
    cart_via1 = np.array(start)
    cart_via1[2] += 0.04
    # print(cart_via1)
    cart_place = np.array(goal)  # place 해야 하는 곳의 cartesian
    # print(cart_place)
    cart_via2 = np.array(goal)
    cart_via2[2] += 0.04
    # print(cart_via2)

    """ Cartesian Trajectories """
    # Trajectory 1: Move to Start point horizontally
    traj_s1, t1 = Trajectory.LSPB(q0=cart_home, qf=cart_via1, tf=tf, tb=tf / 3)

    # Trajectory 2: Go down for Gripping
    traj_s2, t2 = Trajectory.LSPB(q0=cart_via1, qf=cart_pick, tf=tf, tb=tf / 3)

    # Trajectory 3: Pick up
    traj_s3, t3 = Trajectory.LSPB(q0=cart_pick, qf=cart_via1, tf=tf, tb=tf / 3)

    # Trajectory 4: Move to Goal point horizontally
    traj_s4, t4 = Trajectory.LSPB(q0=cart_via1, qf=cart_via2, tf=tf, tb=tf / 3)

    # Trajectory 5: Place
    traj_s5, t5 = Trajectory.LSPB(q0=cart_via2, qf=cart_place, tf=tf, tb=tf / 3)

    # Trajectory 6: Return
    traj_s6, t6 = Trajectory.LSPB(q0=cart_place, qf=cart_via2, tf=tf, tb=tf / 3)

    # Trajectory 7: Home coming
    traj_s7, t7 = Trajectory.LSPB(q0=cart_via2, qf=cart_home, tf=tf, tb=tf / 3)

    """ Joint Trajectories using IK """

    move_joint(traj_s1, t1, "1", cart_pick, gripAngle, ungripAngle)

    move_joint(traj_s2, t2, "2", cart_pick, gripAngle, ungripAngle, 1)

    move_joint(traj_s3, t3, "3", cart_pick, gripAngle, ungripAngle)

    move_joint(traj_s4, t4, "4", cart_pick, gripAngle, ungripAngle)

    move_joint(traj_s5, t5, "5", cart_pick, gripAngle, ungripAngle, 2)

    move_joint(traj_s6, t6, "6", cart_pick, gripAngle, ungripAngle)

    move_joint(traj_s7, t7, "7", cart_pick, gripAngle, ungripAngle)


def move_joint(traj_s, t, pos_num, cart_pick, gripAngle, ungripAngle, grip=0):
    print("move position " + pos_num)
    for i in range(len(t)):
        start_time = time.time()
        q_cur = np.zeros(3)
        for j in range(3):
            q_cur[j] = dxls.get_pos(dxl_ids[j])
        q_cur = dxlPos2rad(q_cur)
        dxl_pos1 = rad2dxlPos(fsolve(ik, x0=q_cur, args=tuple(traj_s[i])))
        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos1[k]))

        print("time: ", time.time() - start_time, "dxl_pos1: ", dxl_pos1, "traj_s1 :", traj_s[i])

        if grip == 1:
            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian

            ''' Grip '''
            if np.linalg.norm(cart_cur - cart_pick) < 10e-4:
                dxls.set_pos(dxl_ids[3], gripAngle)

        elif grip == 2:
            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian

            ''' Place '''
            if np.linalg.norm(cart_cur - cart_pick) < 10e-4:
                dxls.set_pos(dxl_ids[3], ungripAngle)
        else:
            pass


def dxlPos2rad(pos=[]):
    # dxl position: 0 ~ 4095 (0 degree at 2048)
    joints = 2 * np.array(pos) * np.pi / 4096
    return joints


def rad2dxlPos(joints=[]):
    pos = 4096 * np.array(joints) / (2 * np.pi)
    return pos


def homePosition():
    homePos = [2048, 2048, 2048, 2048]
    dxls.set_pos_sync(dxl_ids, homePos)
    # time.sleep(1)
    # print(dxls.get_pos_sync(dxl_ids))


cart_start = [0.18, 0.15, 0.02]
cart_goal = [0.18, -0.15, 0.02]

HomeFlag = False
pickNplace(start=cart_start, goal=cart_goal)
HomeFlag = True
time.sleep(2)
homePosition()
