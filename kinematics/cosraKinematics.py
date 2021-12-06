import numpy as np
from numpy import sin, cos

import cosraVar

np.set_printoptions(precision=5, suppress=True)


class cosraKinematics:
    def __init__(self):
        self.q0_null = np.array([0.0, 0.0, 0.0, 0.0])

    @classmethod
    def DH_transform(cls, dhparams):  # stacks transforms of neighbor frame, following the modified DH convention
        Ts = [np.array([[np.cos(theta), -np.sin(theta), 0, a],
                        [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha),
                         -np.sin(alpha) * d],
                        [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha), np.cos(alpha),
                         np.cos(alpha) * d],
                        [0, 0, 0, 1]]) for [alpha, a, d, theta] in dhparams]
        return Ts

    @classmethod
    def fk(cls, joints):
        Ts = cosraKinematics.DH_transform(cosraVar.dhparam(joints))  # Tb1, T12, T23, ...
        # Tbe = np.linalg.multi_dot(Ts)   # from base to end-effector
        Tbs = np.array(
            [np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, len(Ts) + 1)])  # Tb1, Tb2, Tb3, ...
        # Tbs[-1]: from base to end effector
        return Tbs[-1]

    @classmethod
    def ik(cls, q, *cart_des):
        x, y, z = cart_des
        # print(x)

        q1, q2, q3 = q
        F = np.empty((3))

        F[0] = d3 * sin(q1) + l1 * cos(q1) + l2 * cos(q1) * cos(q2) + l3 * (-sin(q2) * sin(q3) * cos(q1) + cos(q1) * cos(q2) * cos(q3)) - x
        F[1] = -d3 * cos(q1) + l1 * sin(q1) + l2 * sin(q1) * cos(q2) + l3 * (-sin(q1) * sin(q2) * sin(q3) + sin(q1) * cos(q2) * cos(q3)) - y
        F[2] = l2 * sin(q2) + l3 * (sin(q2) * cos(q3) + sin(q3) * cos(q2)) - z
        # print(q)
        return F  # return required joint angles [rad]

    # @classmethod
    # def fk_matrix_to_vector(cls, T):
    #     pos = T[:3, -1]
    #     rot = Rotation.from_matrix(T[:3, :3]).as_rotvec()  # for Python 3
    #     return np.concatenate((pos, rot))

    # @classmethod
    # def fk_vector_to_matrix(cls, x):
    #     T = np.zeros((4, 4))
    #     T[:3, -1] = x[:3]
    #     T[:3, :3] = Rotation.from_rotvec(x[3:]).as_matrix()
    #     return T

    # @classmethod
    # def jacobian(cls, Tbs):
    #     """
    #     Tbs: Tb0, Tb1, Tb2, ...
    #     """
    #     Tbe = Tbs[-1]
    #     J = np.zeros((6, 7))
    #     for i in range(7):
    #         Zi = Tbs[i, :3, 2]  # vector of actuation axis
    #         J[3:, i] = Zi  # Jw
    #         Pin = (Tbe[:3, -1] - Tbs[i, :3, -1])  # pos vector from (i) to (n)
    #         J[:3, i] = np.cross(Zi, Pin)  # Jv
    #     return J

    # Using matrix calculation to get J (similar computing speed but maybe faster to get multiple jacobian at once)
    # @classmethod
    # def jacobian(cls, joints):
    #     Tbe, Ts = cosraKinematics.fk(joints)
    #     J = np.zeros((6, 7))
    #     Tbi = np.array([np.linalg.multi_dot(Ts[:i]) if i > 1 else Ts[0] for i in range(1, 8)])
    #     Zi = Tbi[:, :3, 2]   # vector of actuation axis
    #     J[3:] = Zi.T  # Jw
    #     Pin = (Tbe[:3, -1] - Tbi[:, :3, -1])  # pos vector from (i) to (n)
    #     J[:3] = np.cross(Zi, Pin).T  # Jv
    #     return J

    # @classmethod
    # def ik(cls, Tb_ed, q0=[], RRMC=False):  # inverse kinematics using Newton-Raphson Method
    #     assert Tb_ed.shape == (4, 4)
    #     st = time.time()
    #     if q0 == []:
    #         qk = np.array([0.0, 0.0, 0.0])  # initial guess
    #     else:
    #         qk = np.array(q0)
    #     iter = 1
    #     reached = False
    #     while not reached:
    #         Tbs = cosraKinematics.fk(joints=qk)[0]
    #
    #         # Define Cartesian error
    #         Tb_ec = Tbs[-1]  # base to current ee
    #         Tec_ed = np.linalg.inv(Tb_ec).dot(Tb_ed)  # transform from current ee to desired ee
    #         pos_err = Tb_ec[:3, :3].dot(Tec_ed[:3, -1])  # pos err in the base frame
    #         # rot_err = Tb_ec[:3, :3].dot(Rotation.from_dcm(Tec_ed[:3, :3]).as_rotvec())  # rot err in the base frame
    #         rot_err = Tb_ec[:3, :3].dot(Rotation.from_matrix(Tec_ed[:3, :3]).as_rotvec())  # rot err in the base frame
    #         err_cart = np.concatenate((pos_err, rot_err))
    #
    #         # Inverse differential kinematics (Newton-Raphson method)
    #         J = cosraKinematics.jacobian(Tbs)
    #         Jp = np.linalg.pinv(J)
    #         k = 0.5  # step size is scaled down
    #         qk_next = qk + Jp.dot(err_cart * k)
    #         qk = qk_next
    #
    #         # Convergence condition
    #         if np.linalg.norm(err_cart) < 10e-4:
    #             reached = True
    #         else:
    #             iter += 1
    #         if RRMC:
    #             reached = True
    #
    #     print("iter=", iter, "time=", time.time() - st)
    #     assert ~np.isnan(qk).any()
    #     return qk

    # @classmethod
    # def partial_derivative(cls, func, var=0, point=[]):
    #     args = point[:]
    #
    #     def wraps(x):
    #         args[var] = x
    #         return func(*args)
    #
    #     return scipy.misc.derivative(wraps, point[var], dx=1e-6)


if __name__ == "__main__":
    # # FK
    # joints = [0.95, -0.702, -0.678]
    # Tbe = cosraKinematics.fk(joints=joints)[0][-1]
    #
    # # IK
    # import time
    #
    # st = time.time()
    # print("q_des=", joints)
    # print("q_ik =", cosraKinematics.ik(Tb_ed=Tbe))
    # print("t_comp=", time.time() - st)
    pass
