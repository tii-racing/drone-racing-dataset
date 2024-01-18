from trajectory import Trajectory
import math
import pandas as pd
import numpy as np

class Ellipse(Trajectory):
    def __init__(self, center, phi0, phi1, radius, x_axis, y_axis, v0, v1, hz):
        super().__init__(df=pd.DataFrame())

        phi_total = phi1 - phi0
        direction = 1 if phi_total > 0 else -1
        omega0 = direction * v0 / radius
        omega1 = direction * v1 / radius

        total_time = 2 * phi_total / (omega1 + omega0)
        dt = 1 / hz
        n = int(total_time / dt)
        t = np.array([dt * x for x in range(n)] + [total_time])
        t_mid = total_time / 2
        idx = t > t_mid
        t2 = t[idx] - t_mid

        jerk = 4 * (omega1 - omega0) / total_time ** 2
        alpha = jerk * t
        alpha_mid = jerk * t_mid
        alpha[idx] = alpha_mid - jerk * t2
        omega = omega0 + .5 * jerk * t ** 2
        omega_mid = omega0 + .5 * jerk * t_mid ** 2
        omega[idx] = omega_mid + alpha_mid * t2 - .5 * jerk * t2 ** 2
        phi = phi0 + omega0 * t + (1 / 6) * jerk * t ** 3
        phi_mid = phi0 + omega0 * t_mid + (1 / 6) * jerk * t_mid ** 3
        phi[idx] = phi_mid + omega_mid * t2 + .5 * alpha_mid * t2 ** 2 - (1 / 6) * jerk * t2 ** 3

        x = x_axis * radius * np.cos(phi) + center[0]
        y = y_axis * radius * np.sin(phi) + center[1]
        z = center[2]

        self.df["t"] = t
        self.df["p_x"] = x
        self.df["p_y"] = y
        self.df["p_z"] = z
        self.add_gradients()
        self.compute_total()

if __name__ == "__main__":
    CENTER = [0., 0., .7]
    PHI0 = -math.pi/2
    PHI1 = math.pi*3/2
    RADIUS = 1.5
    X_AXIS = 6.
    Y_AXIS = 1.
    TOP_SPEED = 3.8 # m/s = TOP_SPEED*X_AXIS
    HZ = 150
    CSV = f"double_ellipse"

    traj = Ellipse(CENTER, PHI0, PHI1, RADIUS, X_AXIS, Y_AXIS, 0, TOP_SPEED, HZ)
    traj += Ellipse(CENTER, PHI0, PHI1, RADIUS, X_AXIS, Y_AXIS, TOP_SPEED, 0, HZ)
    traj.add_heading()
    traj.save(CSV)
    traj.plot(title="Ellipse")