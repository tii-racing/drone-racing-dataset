import numpy as np
from transforms3d.quaternions import rotate_vector, qconjugate, mat2quat, qmult
from transforms3d.utils import normalized_vector


def ctbr_controller(tar_pos, tar_vel, tar_acc, cur_pos, cur_vel, cur_att):
    K_P = np.array([3., 3., 8.])
    K_D = np.array([2.5, 2.5, 5.])
    K_RATES = np.array([5., 5., 1.])
    G = np.array([.0, .0, -9.8])
    FF = .4
    P = tar_pos - cur_pos
    D = tar_vel - cur_vel
    tar_acc = K_P * P + K_D * D - G + FF * tar_acc
    norm_thrust = np.dot(tar_acc, rotate_vector([.0, .0, 1.], cur_att))
    # Calculate target attitude
    z_body = normalized_vector(tar_acc)
    x_body = normalized_vector(np.cross(np.array([.0, 1., .0]), z_body))
    y_body = normalized_vector(np.cross(z_body, x_body))
    tar_att = mat2quat(np.vstack([x_body, y_body, z_body]).T)
    # Calculate body rates
    q_error = qmult(qconjugate(cur_att), tar_att)
    body_rates = 2 * K_RATES * q_error[1:]
    if q_error[0] < 0:
        body_rates = -body_rates
    return norm_thrust, *body_rates

def ctbr2beta(thrust, roll, pitch, yaw):
    MIN_CHANNEL = 1000
    MAX_CHANNEL = 2000
    MAX_RATE = 360
    MAX_THRUST = 40.9
    mid = (MAX_CHANNEL + MIN_CHANNEL) / 2
    d = (MAX_CHANNEL - MIN_CHANNEL) / 2
    thrust = thrust / MAX_THRUST * d * 2 + MIN_CHANNEL
    rates = np.array([roll, pitch, -yaw])
    rates = rates / np.pi * 180 / MAX_RATE * d + mid
    thrust = np.clip(thrust, MIN_CHANNEL, MAX_CHANNEL)
    rates = np.clip(rates, MIN_CHANNEL, MAX_CHANNEL)
    return thrust, *rates