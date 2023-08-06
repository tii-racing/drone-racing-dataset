import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

mocap_data = pd.read_csv("drone_state.csv")

fig = plt.figure(figsize=(16,8))
ax = fig.add_subplot(projection='3d')
speed = np.sqrt(mocap_data["velocity_linear_x"]**2 + mocap_data["velocity_linear_y"]**2 + mocap_data["velocity_linear_z"]**2)
ax.scatter(mocap_data["pose_position_x"], mocap_data["pose_position_y"], mocap_data["pose_position_z"], s=1, c = speed, cmap = 'coolwarm')
ax.set_box_aspect(aspect = (12,3,1))
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')
ax.xaxis.labelpad=25
plt.tight_layout()

plt.show()