import numpy as np

""" DH parameters """
# link lengths a(i-1) [m]
l1 = 0.095 # J1 ~ J2
l2 = 0.150 # J2 ~ J3
l3 = 0.150 # J3 ~ EE
lg = 0.140 # gripper body length
# lt = 0.045  # gripper tip length
hb = 0.040    # base height


def dhparam(joints):    # dh parameters [alpha, a, d, theta]
    q1, q2, q3 = np.array(joints).T
    # dynamixel zero position (180degree) & initial configuration compensation
    return np.array([[0,          0,  0, q1 - np.pi],
                     [np.pi / 2, l1,  0, q2 - np.pi + 30 * np.pi / 180],
                     [-np.pi,    l2,  0, q3 - np.pi - 50 * np.pi / 180],
                     [0,         l3,  0,  0]])


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
