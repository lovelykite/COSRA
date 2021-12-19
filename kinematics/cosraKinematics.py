import numpy as np
from numpy import sin, cos

# from cosra import *
import cosraVar
from cosraVar import l1, l2, l3

np.set_printoptions(precision=5, suppress=True)


class cosraKinematics:
    def __init__(self):
        self.q0_null = np.array([0.0, 0.0, 0.0])

    @classmethod
    def DH_transform(cls, dhparams):  # stacks transforms of neighbor frame, following the modified DH convention
        Ts = [np.array([[np.cos(theta), -np.sin(theta), 0, a],
                        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),
                         -np.sin(alpha) * d],
                        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha),
                         np.cos(alpha) * d],
                        [0, 0, 0, 1]]) for [alpha, a, d, theta] in dhparams]
        return Ts

    ''' Forward Kinematics '''
    @classmethod
    def fk(cls, joints):
        Ts = cosraKinematics.DH_transform(cosraVar.dhparam(joints))  # T01, T12, T23, T3e
        # Tbe = np.linalg.multi_dot(Ts)   # from base to end-effector
        Tbs = np.array(
            [np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, len(Ts) + 1)])  # Tb1, Tb2, Tb3, ...
        # Tbs[-1]: from base to end effector
        return Tbs[-1]

    ''' Inverse Kinematics: Both analytical & geometrical solution are valid '''
    # Analytical solution
    @classmethod
    def ik(cls, q, *cart_des):
        x, y, z = cart_des
        # print("x coord: ", x, " y coord: ", y, " z coord: ", z)

        q1, q2, q3 = q
        
        # Compensation for dynamixel initial position and initial manipulator configuration
        q1 -= np.pi
        q2 = q2 - np.pi + 30 * np.pi / 180
        q3 = q3 - np.pi - 50 * np.pi / 180

        F = np.empty((3))

        F[0] = l1 * cos(q1) + l2 * cos(q1) * cos(q2) + l3 * (sin(q2) * sin(q3) * cos(q1) + cos(q1) * cos(q2) * cos(q3)) - x
        F[1] = l1 * sin(q1) + l2 * sin(q1) * cos(q2) + l3 * (sin(q1) * sin(q2) * sin(q3) + sin(q1) * cos(q2) * cos(q3)) - y
        F[2] = l2 * sin(q2) + l3 * (sin(q2) * cos(q3) - sin(q3) * cos(q2)) - z
        # print("q1: ", q1, "q2: ", q2, "q3: ", q3,)
        return F  # return required joint angles [rad]

    # Geometrical solution
    @classmethod
    def ik_geo(cls, q, *cart_des):
        x, y, z = cart_des

        q1, q2, q3 = q

        # Compensation for dynamixel initial position and initial manipulator configuration
        q1 -= np.pi
        q2 = q2 - np.pi + 30 * np.pi / 180
        q3 = q3 - np.pi - 50 * np.pi / 180

        F = np.empty((3))

        F[0] = (l1 + l2 * cos(q2) + l3 * cos(q3 - q2)) * cos(q1) - x
        F[1] = (l1 + l2 * cos(q2) + l3 * cos(q3 - q2)) * sin(q1) - y
        F[2] = l2 * sin(q2) - l3 * sin(q3 - q2) - z

        return F

if __name__ == "__main__":
    pass
