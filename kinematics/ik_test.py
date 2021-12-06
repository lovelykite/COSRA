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
l1 = 0.080  # J1 ~ J2
l2 = 0.136  # J2 ~ J3
l3 = 0.150  # J3 ~ ee
lg = 0.023  # gripper height
h = 0.04  # base height

# link offset di
d3 = 0.055  # joint3 offset
beta = np.arctan2(d3, l2)

# joint angle qi
q = [3.0987, 0.3068, 0.9203]


def dhparam(joints):
    q1, q2, q3 = np.array(joints).T
    # dh parameters [alpha, a, d, theta]
    return np.array([[0, 0, 0, q1],
                     [np.pi / 2, l1, 0, q2],
                     [0, l2, d3, q3],
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


# print(fk(q))

# q_cur = [np.pi / 3, np.pi / 3, np.pi / 3]

### cartesian coordinates ~ joint angle
# px = d3*sin(q1) + l1*cos(q1) + l2*cos(q1)*cos(q2) + l3*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))
# py = -d3*cos(q1) + l1*sin(q1) + l2*sin(q1)*cos(q2) + l3*(-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))
# pz = l2*sin(q2) + l3*(sin(q2)*cos(q3) + sin(q3)*cos(q2))

def ik(q, *cart_des):
    x, y, z = cart_des
    # x = cart_des[0]
    # print(x)
    # y = cart_des[1]
    # print(y)
    # z = cart_des[2]
    # print(z)

    q1, q2, q3 = q
    F = np.empty((3))

    F[0] = d3 * sin(q1) + l1 * cos(q1) + l2 * cos(q1) * cos(q2) + l3 * (
            -sin(q2) * sin(q3) * cos(q1) + cos(q1) * cos(q2) * cos(q3)) - x
    F[1] = -d3 * cos(q1) + l1 * sin(q1) + l2 * sin(q1) * cos(q2) + l3 * (
            -sin(q1) * sin(q2) * sin(q3) + sin(q1) * cos(q2) * cos(q3)) - y
    F[2] = l2 * sin(q2) + l3 * (sin(q2) * cos(q3) + sin(q3) * cos(q2)) - z
    # print(q)
    return F


# cart_des = (8.41313972e-02, 3.57198545e-02, 2.47683265e-01)
# q_cur = [0.96, 0.96, 0.96]
# qk = fsolve(ik(cart_des), q_cur)
# q = fsolve(ik, x0=q_cur, args=cart_des)
# print(q)


def pickNplace(start=[], goal=[]):


    t = 0.0
    tf = 3.0
    gripAngle = 333  # 그리퍼가 물체 잡기 위해 필요한 각도 (0~4095)

    # Via points
    q_home = dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))  # 각 id별 dxl position을 받아 rad으로 변환
    # print(q_home)
    cart_home = cosraKinematics.fk(joints=q_home)[:3, -1]  # 현재 ee cartesian
    # print(cart_home)

    cart_pick = np.array(start)  # pick 해야 하는 물체의 cartesian
    # print(cart_pick)
    # print(start[:2])
    # print(start[2])
    # print(start[2] + 0.04)
    cart_via1 = np.array(start)
    cart_via1[2] += 0.04
    # cart_via1 = np.concatenate((start[:2], np.array(start[2] + 0.04)))
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
    inHome = False

    for i in range(len(t1)):
        start_time = time.time()
        # print(traj_s1)
        # print(tuple(traj_s1[i]))
        # traj = tuple(traj_s1[i])
        # print(traj)
        q_cur1 = np.zeros(3)
        for j in range(3):
            q_cur1[j] = dxls.get_pos(dxl_ids[j])
        # q_cur = dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))
        q_cur1 = dxlPos2rad(q_cur1)
        # print(q_cur1)
        joints1 = fsolve(ik, x0=q_cur1, args=tuple(traj_s1[i]))
        # joints1 = fsolve(ik, x0=q_cur, args=traj)
        # print(joints1)
        # joints1 = cosraKinematics.ik(traj_s1[i])  # radians
        dxl_pos1 = rad2dxlPos(joints1)
        # print(dxl_pos1)
        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos1[k]))

        print("time: ", time.time() - start_time, "dxl_pos1: ", dxl_pos1, "traj_s1 :", traj_s1[i])

        # dxls.set_pos_sync(dxl_ids[:3], dxl_pos1)
        # time.sleep(0.5)

        # for u in range(3):
            # print(dxls.get_pos(dxl_ids[u]))

    for i in range(len(t2)):
        q_cur2 = np.zeros(3)

        for j in range(3):
            q_cur2[j] = dxls.get_pos(dxl_ids[j])

        q_cur2 = dxlPos2rad(q_cur2)
        joints2 = fsolve(ik, x0=q_cur2, args=tuple(traj_s2[i]))
        dxl_pos2 = rad2dxlPos(joints2)

        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos2[k]))

        cart_cur2 = cosraKinematics.fk(joints=q_cur2)[:3, -1]  # home position EE cartesian

        ''' Grip '''
        if np.linalg.norm(cart_cur2 - cart_pick) < 10e-4:
            dxls.set_pos(dxl_ids[3], gripAngle)
            # time.sleep(2)

    # time.sleep(0.5)

    for i in range(len(t3)):
        q_cur = np.zeros(3)

        for j in range(3):
            q_cur[j] = dxls.get_pos(dxl_ids[j])

        q_cur = dxlPos2rad(q_cur)
        joints = fsolve(ik, x0=q_cur, args=tuple(traj_s3[i]))
        dxl_pos = rad2dxlPos(joints)

        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

    for i in range(len(t4)):
        q_cur = np.zeros(3)

        for j in range(3):
            q_cur[j] = dxls.get_pos(dxl_ids[j])

        q_cur = dxlPos2rad(q_cur)
        joints = fsolve(ik, x0=q_cur, args=tuple(traj_s4[i]))
        dxl_pos = rad2dxlPos(joints)

        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

    for i in range(len(t5)):
        q_cur = np.zeros(3)

        for j in range(3):
            q_cur[j] = dxls.get_pos(dxl_ids[j])

        q_cur = dxlPos2rad(q_cur)
        joints = fsolve(ik, x0=q_cur, args=tuple(traj_s5[i]))
        dxl_pos = rad2dxlPos(joints)

        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

        cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian

        ''' Place '''
        if np.linalg.norm(cart_cur - cart_pick) < 10e-4:
            dxls.set_pos(dxl_ids[3], -1 * gripAngle)
            # time.sleep(2)

    for i in range(len(t6)):
        q_cur = np.zeros(3)

        for j in range(3):
            q_cur[j] = dxls.get_pos(dxl_ids[j])

        q_cur = dxlPos2rad(q_cur)
        joints = fsolve(ik, x0=q_cur, args=tuple(traj_s6[i]))
        dxl_pos = rad2dxlPos(joints)

        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

    for i in range(len(t7)):
        q_cur = np.zeros(3)

        for j in range(3):
            q_cur[j] = dxls.get_pos(dxl_ids[j])

        q_cur = dxlPos2rad(q_cur)
        joints = fsolve(ik, x0=q_cur, args=tuple(traj_s7[i]))
        dxl_pos = rad2dxlPos(joints)

        for k in range(3):
            dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

        cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian

        ''' Place '''
        if np.linalg.norm(cart_cur - cart_home) < 10e-4:
            inHome = True
            # time.sleep(2)
            return inHome

def move_joint():


def move_grip():


def dxlPos2rad(pos=[]):
    # dxl position: 0 ~ 4095 (0 degree at 2048)
    joints = np.zeros(3)
    for i in range(3):
        joints[i] = 2 * pos[i] * np.pi / 4096
    return joints


def rad2dxlPos(joints=[]):
    pos = np.zeros(3)
    for i in range(3):
        pos[i] = 4096 * joints[i] / (2 * np.pi)
    return pos


def homePosition():
    homePos = [2048, 2048, 2048, 2048]
    dxls.set_pos_sync(dxl_ids, homePos)
    # time.sleep(1)
    # print(dxls.get_pos_sync(dxl_ids))


cart_start = [0.2, 0.2, 0.02]
cart_goal = [-0.2, 0.1, 0.02]
pickNplace(start=cart_start, goal=cart_goal)
