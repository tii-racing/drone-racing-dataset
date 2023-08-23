import argparse
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Plotting function
def plot_data(df, subplots):

    # Check which columns are present in the dataframe so as all dataset csv type could be used
    if not "drone_x" in df.columns:
        if "pose_position_x" in df.columns:
            df.rename(columns={"pose_position_x": "drone_x", "pose_position_y": "drone_y", "pose_position_z": "drone_z"}, inplace=True)
            df.rename(columns={"pose_orientation_x": "drone_roll", "pose_orientation_y": "drone_pitch", "pose_orientation_z": "drone_yaw"}, inplace=True)
            df.rename(columns={"velocity_linear_x": "drone_velocity_linear_x", "velocity_linear_y": "drone_velocity_linear_y", "velocity_linear_z": "drone_velocity_linear_z"}, inplace=True)
        else:
            subplots.remove('3d')
            subplots.remove('position')
    if not "accel_x" in df.columns:
        subplots.remove('accel')
    if not "gyro_x" in df.columns:
        subplots.remove('gyro')
    if not "thrust[0]" in df.columns:
        subplots.remove('thrust')
    if not "vbat" in df.columns:
        subplots.remove('vbat')
    if not "drone_roll" in df.columns:
        subplots.remove('rotation')
    if not "channels_roll" in df.columns:
        if "roll" in df.columns:
            df.rename(columns={"roll": "channels_roll", "pitch": "channels_pitch", "thrust": "channels_thrust", "yaw": "channels_yaw"}, inplace=True)
        else:
            subplots.remove('channels')

    # Convert microseconds timestamp to seconds from start
    df['timestamp'] = (df['timestamp'] - df['timestamp'][0]) / 1e6

    fig = plt.figure(figsize=((25, 10)))

    num_rows = math.ceil(len(subplots) / 2.0)
    num_cols = 2 if len(subplots) > 1 else 1
    subplot_count = 1

    # 3D Trajectory and Gate Positions
    if '3d' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count, projection='3d')
        if 'drone_velocity_linear_x' in df.columns:
            speed = np.sqrt(df["drone_velocity_linear_x"]**2 + df["drone_velocity_linear_y"]**2 + df["drone_velocity_linear_z"]**2)
        else:
            speed = np.zeros(df.shape[0])
        ax.scatter(df['drone_x'], df['drone_y'], df['drone_z'], label='Drone Trajectory', s=1, c = speed, cmap = 'coolwarm')
        ax.set_box_aspect(aspect = (12,3,1))
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.xaxis.labelpad=25
        if 'gate1_marker1_x' in df.columns:
            gate_markers = ['gate1', 'gate2', 'gate3', 'gate4']
            for gate in gate_markers:
                # Extract coordinates for each marker for the first time instance
                points = np.array([[df.loc[0, f'{gate}_marker{i}_x'], 
                                    df.loc[0, f'{gate}_marker{i}_y'], 
                                    df.loc[0, f'{gate}_marker{i}_z']] for i in range(1, 5)])

                # Compute the centroid
                centroid = np.mean(points, axis=0)
                # Compute the angle each point makes with the centroid
                angles = np.arctan2(points[:,1] - centroid[1], points[:,0] - centroid[0])
                # Sort points by angle
                sorted_points = points[np.argsort(angles)]
                # Complete the loop by appending the first marker at the end
                x_vals, y_vals, z_vals = zip(*(list(sorted_points) + [sorted_points[0]]))
                
                # Draw rectangle
                ax.plot(x_vals, y_vals, z_vals, label=f'{gate}_rectangle')
        ax.set_title('3D Trajectory and Gate Positions')
        subplot_count += 1

    # Acceleration
    if 'accel' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count)
        ax.plot(df['timestamp'], df['accel_x'], label='Accel X')
        ax.plot(df['timestamp'], df['accel_y'], label='Accel Y')
        ax.plot(df['timestamp'], df['accel_z'], label='Accel Z')
        ax.set_title('Acceleration [m/s^2]')
        ax.legend()
        subplot_count += 1

    # Gyroscope
    if 'gyro' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count)
        ax.plot(df['timestamp'], df['gyro_x'], label='Gyro X')
        ax.plot(df['timestamp'], df['gyro_y'], label='Gyro Y')
        ax.plot(df['timestamp'], df['gyro_z'], label='Gyro Z')
        ax.set_title('Gyroscope [rad/s]')
        ax.legend()
        subplot_count += 1

    # Thrust
    if 'thrust' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count)
        ax.plot(df['timestamp'], df['thrust[0]'], label='Thrust 0')
        ax.plot(df['timestamp'], df['thrust[1]'], label='Thrust 1')
        ax.plot(df['timestamp'], df['thrust[2]'], label='Thrust 2')
        ax.plot(df['timestamp'], df['thrust[3]'], label='Thrust 3')
        ax.set_title('Normalized Thrust [0, 1]')
        ax.legend()
        subplot_count += 1

    # Battery Voltage
    if 'vbat' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count)
        ax.plot(df['timestamp'], df['vbat'])
        ax.set_title('Battery Voltage [V]')
        subplot_count += 1

    # Drone Position
    if 'position' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count)
        ax.plot(df['timestamp'], df['drone_x'], label='X')
        ax.plot(df['timestamp'], df['drone_y'], label='Y')
        ax.plot(df['timestamp'], df['drone_z'], label='Z')
        ax.set_title('Position [m]')
        ax.legend()
        subplot_count += 1

    # Drone Rotation
    if 'rotation' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count)
        ax.plot(df['timestamp'], df['drone_roll'], label='Roll')
        ax.plot(df['timestamp'], df['drone_pitch'], label='Pitch')
        ax.plot(df['timestamp'], df['drone_yaw'], label='Yaw')
        ax.set_title('Rotation [rad]')
        ax.legend()
        subplot_count += 1

    # Channels
    if 'channels' in subplots:
        ax = fig.add_subplot(num_rows, num_cols, subplot_count)
        ax.plot(df['timestamp'], df['channels_roll'], label='Roll')
        ax.plot(df['timestamp'], df['channels_pitch'], label='Pitch')
        ax.plot(df['timestamp'], df['channels_thrust'], label='Thrust')
        ax.plot(df['timestamp'], df['channels_yaw'], label='Yaw')
        ax.set_title('Channels BF [1000, 2000]')
        ax.legend()
        subplot_count += 1

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visualize drone CSV data.')
    parser.add_argument('--csv-file', required=True, help='Path to the CSV file')
    parser.add_argument('--subplots', nargs='+', default=['3d', 'accel', 'gyro', 'thrust', 'vbat', 'position', 'rotation', 'channels'],
                        help='Specify which subplots to show.')
    args = parser.parse_args()

    df = pd.read_csv(args.csv_file)
    plot_data(df, args.subplots)
