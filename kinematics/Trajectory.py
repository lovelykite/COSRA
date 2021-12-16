import numpy as np


class Trajectory:
    def __init__(self):
        pass

    @classmethod
    def cubic(cls, q0, qf, v0, vf, tf, t_step=0.01):
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)
        if np.allclose(q0, qf):
            t = [0.0]
            q_pos = [qf]
            return q_pos, t

        # Define coefficients
        a0 = q0
        a1 = v0
        a2 = 3*(qf-q0)/(tf**2) - 2/tf*v0 - 1/tf*vf
        a3 = -2*(qf-q0)/(tf**3) + 1/(tf**2)*(v0+vf)

        # Calculate trajectories
        t = np.arange(start=0.0, stop=tf, step=t_step).reshape(-1, 1)

        # joint traj.
        q_pos = a0 + a1*t + a2*t**2 + a3*t**3
        q_vel = a1 + 2*a2*t + 3*a3*t**2
        q_acc = 2*a2 + 6*a3*t
        assert ~np.isnan(t).any()
        assert ~np.isnan(q_pos).any()
        return q_pos, t

    @classmethod
    def LSPB(cls, q0, qf, tf, tb, t_step=0.07):
        q0 = np.array(q0)
        qf = np.array(qf)
        if np.allclose(q0, qf):
            t = [0.0]
            q_pos = [qf]
            return q_pos, t

        # Define coefficients
        ab = (q0 - qf) / (tb - tf) / tb
        A = np.array([[tb, -tb, 0.0, 1.0, -1.0, 0.0],
                      [0.0, -(tf - tb), tf - tb, 0.0, -1.0, 1.0],
                      [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, tf, 0.0, 0.0, 1.0]])
        b = np.block([[-ab * tb ** 2 / 2], [ab * (tf - tb) ** 2 / 2], [np.zeros_like(q0)], [q0], [ab * tf],
                      [qf + ab * tf ** 2 / 2]])
        coeff = np.linalg.inv(A).dot(b)
        C1 = coeff[0]
        C2 = coeff[1]
        C3 = coeff[2]
        C4 = coeff[3]
        C5 = coeff[4]
        C6 = coeff[5]

        # Calculate trajectories
        t = np.arange(start=0.0, stop=tf, step=t_step)
        t1 = t[t < tb].reshape(-1, 1)
        t2 = t[(tb <= t) & (t < tf - tb)].reshape(-1, 1)
        t3 = t[tf - tb <= t].reshape(-1, 1)

        # Combine joint trajectories
        traj1 = ab / 2 * t1 ** 2 + C1 * t1 + C4
        traj2 = C2 * t2 + C5
        traj3 = -ab / 2 * t3 ** 2 + C3 * t3 + C6
        q_pos = np.concatenate((traj1, traj2, traj3))
        assert ~np.isnan(t).any()
        assert ~np.isnan(q_pos).any()
        return q_pos, t


if __name__ == '__main__':
    pass
