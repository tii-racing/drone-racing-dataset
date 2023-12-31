import argparse
import os
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R

VELOCITY_LOWPASS_FREQ = 100.0  # Hz

def synchronize_dataframes(reference_df, dfs):
    synchronized_dfs = []
    reference_timestamps = reference_df['timestamp'].values

    for df in dfs:
        data_timestamps = df['timestamp'].to_numpy()
        data = df.drop('timestamp', axis=1)
        data_values = data.to_numpy()
        interpolated_data = np.zeros((len(reference_timestamps), data_values.shape[1]))
        for col_index, col in enumerate(data.columns):
            if '_rot' in col:
                if '_rot[0]' in col:
                    # MoCap data is column-wise, so we need to transpose it to get the rotation matrix
                    rot_matrix = R.from_matrix(data_values[:, col_index:col_index+9].reshape(-1, 3, 3).transpose(0, 2, 1))
                    slerp_interp = Slerp(data_timestamps, rot_matrix)
                    interpolated_data[:, col_index:col_index+9] = slerp_interp(reference_timestamps).as_matrix().transpose(0, 2, 1).reshape(-1, 9)
            else:
                interpolator = interp1d(data_timestamps, data_values[:, col_index], axis=0, fill_value="extrapolate")
                interpolated_data[:, col_index] = interpolator(reference_timestamps)
        interpolated_df = pd.DataFrame(interpolated_data, columns=df.columns[1:])
        synchronized_dfs.append(interpolated_df)
    return synchronized_dfs

def add_velocity_columns(df):
    drone_velocity_linear_x = []
    drone_velocity_linear_y = []
    drone_velocity_linear_z = []
    drone_velocity_angular_x = []
    drone_velocity_angular_y = []
    drone_velocity_angular_z = []

    for index, row in df.iterrows():
        if index > 0:
            dt = ( row['timestamp'] - df.at[index - 1, 'timestamp'] ) / 1000000
            linear_vx = calc_linear_velocity(row['drone_x'], df.at[index - 1, 'drone_x'], dt, drone_velocity_linear_x[-1])
            linear_vy = calc_linear_velocity(row['drone_y'], df.at[index - 1, 'drone_y'], dt, drone_velocity_linear_y[-1])
            linear_vz = calc_linear_velocity(row['drone_z'], df.at[index - 1, 'drone_z'], dt, drone_velocity_linear_z[-1])
            # MoCap data is column-wise, so we need to transpose it to get the rotation matrix
            omega_x, omega_y, omega_z = calc_angular_velocity(
                        row[['drone_rot[0]', 'drone_rot[1]', 'drone_rot[2]',
                             'drone_rot[3]', 'drone_rot[4]', 'drone_rot[5]',
                             'drone_rot[6]', 'drone_rot[7]', 'drone_rot[8]']].to_numpy().reshape(3, 3).transpose(),
                        df.loc[index - 1, ['drone_rot[0]', 'drone_rot[1]', 'drone_rot[2]',
                                          'drone_rot[3]', 'drone_rot[4]', 'drone_rot[5]',
                                          'drone_rot[6]', 'drone_rot[7]', 'drone_rot[8]']].to_numpy().reshape(3, 3).transpose(),
                        dt=dt)

            drone_velocity_angular_x.append(omega_x)
            drone_velocity_angular_y.append(omega_y)
            drone_velocity_angular_z.append(omega_z)
            drone_velocity_linear_x.append(linear_vx)
            drone_velocity_linear_y.append(linear_vy)
            drone_velocity_linear_z.append(linear_vz)
        else:
            drone_velocity_linear_x.append(0.0)
            drone_velocity_linear_y.append(0.0)
            drone_velocity_linear_z.append(0.0)
            drone_velocity_angular_x.append(0.0)
            drone_velocity_angular_y.append(0.0)
            drone_velocity_angular_z.append(0.0)

    df.insert(7, 'drone_velocity_linear_x', drone_velocity_linear_x)
    df.insert(8, 'drone_velocity_linear_y', drone_velocity_linear_y)
    df.insert(9, 'drone_velocity_linear_z', drone_velocity_linear_z)
    df.insert(10, 'drone_velocity_angular_x', drone_velocity_angular_x)
    df.insert(11, 'drone_velocity_angular_y', drone_velocity_angular_y)
    df.insert(12, 'drone_velocity_angular_z', drone_velocity_angular_z)

    return df

def calc_linear_velocity(point, prev_point, dt, last_v):
    # Calculate linear velocity and apply lowpass filter
    linear_velocity = (point - prev_point) / dt
    beta = np.exp(-VELOCITY_LOWPASS_FREQ * 2.0 * np.pi * dt)
    filtered_velocity = beta * last_v + (1 - beta) * linear_velocity
    return filtered_velocity

def calc_angular_velocity(rotation_matrix, last_rotation_matrix, dt):
    dRdt = (rotation_matrix - last_rotation_matrix) / dt
    omega = dRdt @ rotation_matrix.T
    return omega[2, 1], omega[0, 2], omega[1, 0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--flight', required=True, help="Flight ID (e.g., flight-01p-ellipse)")
    parser.add_argument('--sync-option', choices=[1, 2], default=1, type=int, help="Sync type (1: freq_sync or 2: cam_ts_sync)")
    parser.add_argument('--freq', default=500, type=int, help="Frequency for sync type 1 (freq_sync)")
    args = parser.parse_args()

    print("Loading and pre-processing CSVs...")

    flight_type = "piloted" if "p-" in args.flight else "autonomous"
    flight_dir = os.path.join("..", "data", flight_type, args.flight)
    csv_dir = os.path.join(flight_dir, "csv_raw")
    rosbag_dir = os.path.join(csv_dir, "ros2bag_dump")
    csv_filenames = ['camera_', 'imu_', 'motors_thrust_', 'channels_', 'battery_', 'mocap_', 'gate_corners_']
    inside_ros_dir = [False, True, True, True, True, False, False]

    dfs = []
    for i, filename in enumerate(csv_filenames):
        root = rosbag_dir if inside_ros_dir[i] else csv_dir
        csv_path = os.path.join(root, filename + args.flight + '.csv')
        df = pd.read_csv(csv_path)
        dfs.append(df)

    camera_df, imu_df, thrust_df, channels_df, battery_df, mocap_df, gate_corners_df = dfs

#####################
# Preprocessing
#####################

    mocap_df.drop('frame', axis=1, inplace=True)
    mocap_df = add_velocity_columns(mocap_df)

    # Rename ambiguous columns
    channels_df.rename(columns={'roll': 'channels_roll', 'pitch': 'channels_pitch', 'thrust': 'channels_thrust', 'yaw': 'channels_yaw'}, inplace=True)
    camera_df.rename(columns={'filename': 'img_filename'}, inplace=True)

#####################
# Syncing Dataframes
#####################

    final_csv_name = args.flight

    if args.sync_option == 2:
        print("Using camera timestamps to interpolate other data")
        reference_df = camera_df
        dfs_to_sync = [imu_df, thrust_df, channels_df, battery_df, mocap_df, gate_corners_df]
        final_csv_name += '_cam_ts_sync'
    elif args.sync_option == 1:
        print("Using frequency " + str(args.freq) + " to interpolate all data")
        first_timestamp = camera_df['timestamp'].iloc[0]
        last_timestamp = camera_df['timestamp'].iloc[-1]
        target_timestamps = np.arange(first_timestamp, last_timestamp, 1.0 / args.freq * 1000000, dtype=np.ulonglong)
        reference_df = pd.DataFrame({'timestamp': target_timestamps})
        dfs_to_sync = [imu_df, thrust_df, channels_df, battery_df, mocap_df, gate_corners_df]
        final_csv_name += "_" + str(args.freq) + 'hz_freq_sync'
    else:
        print("Invalid sync option. Exiting.")
        return

    print("Syncing dataframes...")
    synchronized_dfs = synchronize_dataframes(reference_df, dfs_to_sync)
    synchronized_dfs[2] = synchronized_dfs[2].astype('int') # channels should be integers
    print("Sync complete.")

    if args.sync_option == 2:
        print("Interpolating camera data...")
        camera_df['timestamp'] = camera_df['timestamp'].astype('uint64')
        reference_df = pd.merge_asof(reference_df, camera_df, on='timestamp', direction='nearest')
        print("Interpolation complete.")

#####################
# Saving Dataframes
#####################

    time_column = (reference_df['timestamp'] - reference_df['timestamp'].iloc[0]) / 1000000
    reference_df.insert(0, 'elapsed_time', time_column)
    final_df = pd.concat([reference_df] + synchronized_dfs, axis=1)
    print("Saving final CSV...")
    final_df.to_csv(os.path.join(flight_dir, final_csv_name + '.csv'), index=False)
    print("Final CSV saved.")

if __name__ == "__main__":
    main()
