import time

import numpy as np
from numpy.linalg import inv

from cosraKinematics import cosraKinematics
from Trajectory import Trajectory
from dynamixel import dynamixel


class cosraCtrl:
    def __init__(self):
        pass

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
            dxls.set_pos(dxl_ids[1], 0)
            dxls.set_pos(dxl_ids[2], 0)
            time.sleep(2)
            dxls.set_pos(dxl_ids[1], 4095)
            dxls.set_pos(dxl_ids[2], 4095)
            time.sleep(2)

    def pickNplace(self, start=[], goal=[]):
        t = 0.0
        tf = 3.0
        gripAngle = 333  # 그리퍼가 물체 잡기 위해 필요한 각도 (0~4095)

        # Via points
        q_init = self.dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))  # 각 id별 dxl position을 받아 rad으로 변환
        print(q_init)
        cart_init = cosraKinematics.fk(joints=q_init)[:3, -1]  # 현재 ee cartesian
        print(cart_init)

        cart_pick = np.array(start)  # pick 해야 하는 물체의 cartesian
        cart_via1 = np.concatenate(start[:2], start[2] + 40)
        cart_place = np.array(goal)  # place 해야 하는 곳의 cartesian
        cart_via2 = np.concatenate(goal[:2], goal[2] + 40)

        """ Cartesian Trajectories """
        # Trajectory 1: Move to Start point horizontally
        traj_s1, t1 = Trajectory.LSPB(q0=cart_init, qf=cart_via1, tf=tf, tb=tf / 3)

        # Trajectory 2: Go down for Gripping
        traj_s2, t2 = Trajectory.LSPB(q0=cart_via1, qf=cart_pick, tf=tf, tb=tf / 3)

        # Trajectory 3: Pick up
        traj_s3, t3 = Trajectory.LSPB(q0=cart_pick, qf=cart_via1, tf=tf, tb=tf / 3)

        # Trajectory 4: Move to Goal point
        traj_s4, t4 = Trajectory.LSPB(q0=cart_via1, qf=cart_via2, tf=tf, tb=tf / 3)

        # Trajectory 5: Go down for placing
        traj_s5, t5 = Trajectory.LSPB(q0=cart_via2, qf=cart_place, tf=tf, tb=tf / 3)

        # Trajectory 6: Return
        traj_s6, t6 = Trajectory.LSPB(q0=cart_place, qf=cart_via2, tf=tf, tb=tf / 3)

        """ Joint Trajectories using IK """
        inMotion = True

        for i in range(len(t1)):
            joints1 = cosraKinematics.ik_geo(traj_s1[i])  # radians
            dxl_pos1 = self.rad2dxlPos(joints1)
            dxls.set_pos_sync(dxl_ids, dxl_pos1)
            # time.sleep(0.005)

        for i in range(len(t2)):
            joints2 = cosraKinematics.ik_geo(traj_s2[i])  # radians
            dxl_pos2 = self.rad2dxlPos(joints2)
            dxls.set_pos_sync(dxl_ids, dxl_pos2)
            # time.sleep(0.005)

            q_cur = self.dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))  # 각 id별 dxl position을 받아 rad으로 변환
            print(q_cur)
            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # 현재 ee cartesian
            print(cart_cur)

            # Grasp
            if np.linalg.norm(cart_cur - cart_pick) < 10e-4:
                dxls.set_pos(dxl_ids[3], gripAngle)
                time.sleep(2)

        for i in range(len(t3)):
            joints3 = cosraKinematics.ik_geo(traj_s3[i])  # radians
            dxl_pos3 = self.rad2dxlPos(joints3)
            dxls.set_pos_sync(dxl_ids, dxl_pos3)
            # time.sleep(0.005)

        for i in range(len(t4)):
            joints4 = cosraKinematics.ik_geo(traj_s4[i])  # radians
            dxl_pos4 = self.rad2dxlPos(joints4)
            dxls.set_pos_sync(dxl_ids, dxl_pos4)
            # time.sleep(0.005)

        for i in range(len(t5)):
            joints5 = cosraKinematics.ik_geo(traj_s5[i])  # radians
            dxl_pos5 = self.rad2dxlPos(joints5)
            dxls.set_pos_sync(dxl_ids, dxl_pos5)
            # time.sleep(0.005)

            q_cur = self.dxlPos2rad(dxls.get_pos_sync(dxl_ids[:3]))  # 각 id별 dxl position을 받아 rad으로 변환
            print(q_cur)
            cart_cur = cosraKinematics.fk(joints=q_cur)[:3, -1]  # 현재 ee cartesian
            print(cart_cur)

            # Grasp
            if np.linalg.norm(cart_cur - cart_pick) < 10e-4:
                dxls.set_pos(dxl_ids[3], gripAngle)
                time.sleep(2)

        for i in range(len(t6)):
            joints6 = cosraKinematics.ik_geo(traj_s6[i])  # radians
            dxl_pos6 = self.rad2dxlPos(joints6)
            dxls.set_pos_sync(dxl_ids, dxl_pos6)
            # time.sleep(0.005)

        inMotion = False
        return inMotion

    def dxlPos2rad(self, pos):
        # dxl position: 0 ~ 4095 (0 degree at 2048)
        joints = np.array(len(pos))
        for i in range(3):
            joints[i] = 2 * pos[i] * np.pi / 4096 - 2048
        return joints

    def rad2dxlPos(self, joints):
        pos = np.array(len(joints))
        for i in range(3):
            pos[i] = 4096 * (joints[i] + 2048) / (2 * np.pi)
        return pos

    def homePosition(self):
        homePos = [2048, 2048, 2048, 2048]
        dxls.set_pos_sync(dxl_ids, homePos)
        # time.sleep(1)
        # print(dxls.get_pos_sync(dxl_ids))


if __name__ == '__main__':
    cosra = cosraCtrl()
    dxls = dynamixel()
    dxl_ids = [0, 1, 2, 3]  # ID setting

    for i in range(4):
        dxls.enable_torque(dxl_ids[i], True)  # torque enable
        # dxls.set_pos(dxl_ids[i], 0)  # home position

    # cosra.dynamixelTest()
    # cosra.cableDrivingTest()
    # cosra.pickNplace()

    reached = False  # whether all the blocks are arranged correctly
    inMotion = False  # whether robot is in motion

    while not reached:
        # 상자 분류 완료하지 못 했을 때
        cosra.pickNplace()

        # 상자 분류 완성했을 때
        # if ...:
        #     reached = True
        #     break

    # cosra.homePosition()
    dxls.close_port()
