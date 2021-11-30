import numpy as np

""" DH parameters """
# link lengths [m]
L1 = 0.1360  # J1 ~ J2
L2 = 0.1500  # J2 ~ J3
Lg = 0.0584  # gripper body length
Lt = 0.045  # gripper tip length

# link offsets [m]
d2 = 0.0550  # elbow offset


def dhparam(joints):
    q1, q2, q3 = np.array(joints).T
    return np.array([[0, 0, 0, q1],
                     [np.pi / 2, d2, L1, q2],
                     [0, 0, L2, q3],
                     [0, 0, Lg + Lt, 0]])


# # Joint space limits
# q_max = np.array([ 2.8973,  1.7628,  2.8973, -0.0698])    # joint limit (rad)
# q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718])    # joint limit (rad)
# qv_ratio = 0.8   # safety margin
# qa_ratio = 0.8
# qj_ratio = 0.8
# qv_max = np.array([ 2.1750, 2.1750,  2.1750,  2.1720])*qv_ratio    # max. ang. vel (rad/s)
# qa_max = np.array([15.0000, 7.5000, 10.0000, 12.5000])*qa_ratio    # max. ang. acc (rad/s^2)
# qj_max = np.array([   7500,   3750,    5000,    6250])*qj_ratio    # max. ang. jerk (rad/s^3)
# tau_max = np.array([87, 87, 87, 87])    # max torque (Nm)
