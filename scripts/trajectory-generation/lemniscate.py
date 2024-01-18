from trajectory import Trajectory
import pandas as pd
import numpy as np
from scipy import optimize
from scipy import integrate

class HalfLobe(Trajectory):
    def __init__(self, a, b, v0, v1, hz):
        super().__init__(df=pd.DataFrame())
        self.a = a
        self.b = b

        total_length = self.arclength(np.pi / 2)
        total_time = 2 * total_length / (v1 + v0)
        jerk = 4 * (v1 - v0) / total_time ** 2
        n = int(total_time * hz)
        dt = 1 / hz

        t = np.array([dt * x for x in range(n)])
        t_mid = total_time / 2
        idx = t > t_mid
        t2 = t[idx] - t_mid

        j = jerk * np.ones_like(t)
        j[idx] = -jerk
        acc = jerk * t
        acc_mid = jerk * t_mid
        acc[idx] = acc_mid - jerk * t2
        v = v0 + .5 * jerk * t ** 2
        v_mid = v0 + .5 * jerk * t_mid ** 2
        v[idx] = v_mid + acc_mid * t2 - .5 * jerk * t2 ** 2
        d = v0 * t + (1 / 6) * jerk * t ** 3
        d_mid = v0 * t_mid + (1 / 6) * jerk * t_mid ** 3
        d[idx] = d_mid + v_mid * t2 + .5 * acc_mid * t2 ** 2 - (1 / 6) * jerk * t2 ** 3

        phi = np.array([self.phi_from_arclength(x) for x in d])

        x = self.a * np.cos(phi) / (1 + np.sin(phi) ** 2)
        y = self.a * self.b * np.sin(phi) * np.cos(phi) / (1 + np.sin(phi) ** 2)

        self.df['t'] = t.tolist() + [total_time]
        self.df['p_x'] = x.tolist() + [0]
        self.df['p_y'] = y.tolist() + [0]
        self.df['p_z'] = 0
        self.df['v_x'] = 0
        self.df['v_y'] = 0
        self.df['v_z'] = 0
        self.df['a_lin_x'] = 0
        self.df['a_lin_y'] = 0
        self.df['a_lin_z'] = 0

    def dxdphi(self, phi):
        return (self.a * np.sin(phi) * (np.sin(phi) ** 2 - 3)) / (np.sin(phi) ** 2 + 1) ** 2

    def dydphi(self, phi):
        return self.a * self.b * (3 * np.cos(phi) ** 2 - 2) / (np.sin(phi) ** 2 + 1) ** 2

    def dlengthdphi(self, phi):
        return np.sqrt(self.dxdphi(phi) ** 2 + self.dydphi(phi) ** 2)

    def arclength(self, phi):
        return integrate.quad(lambda x: self.dlengthdphi(x), 0, phi)[0]

    def phi_from_arclength(self, d):
        return optimize.brentq(lambda x: d - self.arclength(x), 0, np.pi / 2)

class Lemniscate(Trajectory):
    def __init__(self, center, a, b, v, hz):
        super().__init__(df=pd.DataFrame())
        lobe = HalfLobe(a, b, v[1], v[0], hz).reverse()
        lobe += HalfLobe(a, b, v[1], v[2], hz).mirror_y()
        lobe += HalfLobe(a, b, v[3], v[2], hz).mirror_x().reverse()
        lobe += HalfLobe(a, b, v[3], v[4], hz).mirror_x().mirror_y()
        lobe.translate(x=center[0], y=center[1], z=center[2])
        self.df = lobe.df.copy()
        self.df.drop_duplicates(subset=['t'], inplace=True)
        self.add_gradients()
        self.compute_total()

if __name__ == "__main__":
    CENTER = [0., 0., .7]
    A = 6.
    B = .708
    SPEED =[0, 4, 8, 6, 10]
    HZ = 150
    CSV = "double_lemniscate"

    traj = Lemniscate(CENTER, A, B, SPEED, HZ)
    traj += Lemniscate(CENTER, A, B, SPEED[::-1], HZ)
    traj.add_heading()
    traj.save(CSV)
    traj.plot("Lemniscate")