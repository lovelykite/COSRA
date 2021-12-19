import numpy as np

""" DH parameters """
# link lengths a(i-1) [m]
l1 = 0.095 # J1 ~ J2
l2 = 0.150 # J2 ~ J3
l3 = 0.150 # J3 ~ EE


def dhparam(joints):    # dh parameters [alpha, a, d, theta]
    q1, q2, q3 = np.array(joints).T
    # Compensation for dynamixel zero position (180 degree) & initial configuration
    return np.array([[0,          0,  0, q1 - np.pi],
                     [np.pi / 2, l1,  0, q2 - np.pi + 30 * np.pi / 180],
                     [-np.pi,    l2,  0, q3 - np.pi - 50 * np.pi / 180],
                     [0,         l3,  0,  0]])
