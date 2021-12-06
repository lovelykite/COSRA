import time

import numpy as np
from numpy.linalg import inv
from scipy.optimize import fsolve

from Trajectory import Trajectory
from cosraKinematics import cosraKinematics
from dynamixel import dynamixel


class cosraCtrl:
    def __init__(self):
        pass

    """ Dynamixel Test: Basic """
    def dynamixelTest(self):
        # checking dynamixel
        # 기구부 구동시키기 위한 모터 출력 체크하기
        for i in range(4):
            dxls.set_pos(dxl_ids[i], 0)
            time.sleep(0.5)

        time.sleep(1)

        for i in range(4):
            dxls.set_pos(dxl_ids[i], 4095)
            time.sleep(0.5)
        dxls.set_pos_sync(dxl_ids, [2048, 2048, 2048, 2048])

    def cableDrivingTest(self):
        while True:
            dxls.set_pos(dxl_ids[3], 1000)
            # dxls.set_pos(dxl_ids[2], 0)
            time.sleep(2)
            dxls.set_pos(dxl_ids[3], 1500)
            # dxls.set_pos(dxl_ids[2], 4095)
            time.sleep(2)

    """ Pick N Place Algorithm """
    def pickNplace(self, start=[], goal=[]):
        t = 0.0
        tf = 3.0        # 시간 조정해야할 수도? 다이나믹셀 팍팍 움직임.
        gripAngle = 333  # 그리퍼가 물체 잡기 위해 필요한 각도 (0 ~ 4095 값 -> tuning)

        # manipulator는 home position에서 수평 방향으로 이동
        # vision 처리를 통해 start object와 goal object 인식하고 좌표에 대한 정보를 보내주면,
        # 해당 좌표의 수직 방향으로 manipulator가 움직이는 평면에서의 via point를 2개 형성.
        # 따라서, manipulator는 home -> via1 -> start -> via1 -> via2 -> goal -> via2 -> home 의 trajectory를 갖는다.

        ''' Home Position '''
        self.homePosition()
        q_home = self.dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))  # 각 id별 dxl position을 받아 rad으로 변환
        # print(q_home)
        cart_home = cosraKinematics.fk(joints=q_home)[:3, -1]  # home position EE cartesian
        # print(cart_home)

        ''' Via Points '''
        cart_pick = np.array(start)  # pick 해야 하는 물체의 horizontal cartesian
        # print(cart_pick)
        cart_via1 = np.array(start)
        cart_via1[2] += 0.04        # manipulator gripper 초기 높이만큼 offset
        # print(cart_via1)
        cart_place = np.array(goal)  # place 해야 하는 곳의 horizontal cartesian
        # print(cart_place)
        cart_via2 = np.array(goal)
        cart_via2[2] += 0.04        # manipulator gripper 초기 높이만큼 offset
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
            # q_cur = self.dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))
            q_cur = np.zeros(3)
            for j in range(3):
                q_cur[j] = dxls.get_pos(dxl_ids[j])
            q_cur = dxlPos2rad(q_cur)
            # print(q_cur)
            joints = fsolve(ik, x0=q_cur, args=tuple(traj_s1[i]))
            # joints1 = cosraKinematics.ik(traj_s1[i])  # radians
            # print(joints1)
            dxl_pos = self.rad2dxlPos(joints)
            # dxls.set_pos_sync(dxl_ids, dxl_pos)
            for k in range(3):
                dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))
            # time.sleep(0.005)

        for i in range(len(t2)):
            q_cur = np.zeros(3)

            for j in range(3):
                q_cur[j] = dxls.get_pos(dxl_ids[j])

            q_cur = dxlPos2rad(q_cur)
            joints = fsolve(ik, x0=q_cur, args=tuple(traj_s2[i]))
            dxl_pos = self.rad2dxlPos(joints)

            for k in range(3):
                dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian

            ''' Grip '''
            if np.linalg.norm(cart_cur - cart_pick) < 10e-4:
                dxls.set_pos(dxl_ids[3], gripAngle)
                time.sleep(2)

        for i in range(len(t3)):
            q_cur = np.zeros(3)

            for j in range(3):
                q_cur[j] = dxls.get_pos(dxl_ids[j])

            q_cur = dxlPos2rad(q_cur)
            joints = fsolve(ik, x0=q_cur, args=tuple(traj_s3[i]))
            dxl_pos = self.rad2dxlPos(joints)

            for k in range(3):
                dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

        for i in range(len(t4)):
            q_cur = np.zeros(3)

            for j in range(3):
                q_cur[j] = dxls.get_pos(dxl_ids[j])

            q_cur = dxlPos2rad(q_cur)
            joints = fsolve(ik, x0=q_cur, args=tuple(traj_s4[i]))
            dxl_pos = self.rad2dxlPos(joints)

            for k in range(3):
                dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

        for i in range(len(t5)):
            q_cur = np.zeros(3)

            for j in range(3):
                q_cur[j] = dxls.get_pos(dxl_ids[j])

            q_cur = dxlPos2rad(q_cur)
            joints = fsolve(ik, x0=q_cur, args=tuple(traj_s5[i]))
            dxl_pos = self.rad2dxlPos(joints)

            for k in range(3):
                dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian

            ''' Place '''
            if np.linalg.norm(cart_cur - cart_pick) < 10e-4:
                dxls.set_pos(dxl_ids[3], -1 * gripAngle)
                time.sleep(2)

        for i in range(len(t6)):
            q_cur = np.zeros(3)

            for j in range(3):
                q_cur[j] = dxls.get_pos(dxl_ids[j])

            q_cur = dxlPos2rad(q_cur)
            joints = fsolve(ik, x0=q_cur, args=tuple(traj_s6[i]))
            dxl_pos = self.rad2dxlPos(joints)

            for k in range(3):
                dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

        for i in range(len(t7)):
            q_cur = np.zeros(3)

            for j in range(3):
                q_cur[j] = dxls.get_pos(dxl_ids[j])

            q_cur = dxlPos2rad(q_cur)
            joints = fsolve(ik, x0=q_cur, args=tuple(traj_s7[i]))
            dxl_pos = self.rad2dxlPos(joints)

            for k in range(3):
                dxls.set_pos(dxl_ids[k], int(dxl_pos[k]))

            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # home position EE cartesian

            ''' Place '''
            if np.linalg.norm(cart_cur - cart_home) < 10e-4:
                inHome = True
                time.sleep(2)
                return inHome

    def dxlPos2rad(self, pos=[]):
        # dxl position: 0 ~ 4095 (0 degree at 2048)
        joints = np.zeros(3)
        for i in range(3):
            joints[i] = 2 * pos[i] * np.pi / 4096
        return joints

    def rad2dxlPos(self, joints=[]):
        pos = np.zeros(3)
        for i in range(3):
            pos[i] = 4096 * joints[i] / (2 * np.pi)
        return pos

    def homePosition(self):
        homePos = [2048, 2048, 2048, 2048]
        dxls.set_pos_sync(dxl_ids, homePos)


if __name__ == '__main__':
    cosra = cosraCtrl()
    dxls = dynamixel()
    dxl_ids = [0, 1, 2, 3]  # ID setting

    for i in range(4):
        dxls.enable_torque(dxl_ids[i], True)  # torque enable
        # dxls.set_pos(dxl_ids[i], 0)  # home position

    # checking current dynamixel position
    # print(dxls.get_pos(dxl_ids[0]))
    # print(cosra.dxlPos2rad(dxls.get_pos(dxl_ids[0])))
    # print(dxls.get_pos(dxl_ids[1]))
    # print(dxls.get_pos(dxl_ids[2]))
    # print(dxls.get_pos_sync(dxl_ids))
    # print(cosra.dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3])))

    # checking fk results: ee cartesian
    # print(cosraKinematics.fk(cosra.dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3])))[:3,-1])

    """ Dynamixel Test: Basic """
    # cosra.dynamixelTest()
    # cosra.cableDrivingTest()

    # reached = False  # whether all the blocks are arranged correctly
    # inHome = False  # whether robot is in motion

    """ Pick N Place Algorithm """
    # while not reached:
    #     # 상자 분류 완료하지 못 했을 때
    #     cosra.pickNplace()

        # 상자 분류 완성했을 때
        # if ...:
        #     reached = True
        #     break

    cosra.homePosition()
    dxls.close_port()
