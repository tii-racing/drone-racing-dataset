import argparse
import pandas as pd
from fastdtw import fastdtw
import numpy as np
from itertools import combinations
import glob

# Function to calculate DTW distance between two trajectories
def calculate_dtw_distance(seq1, seq2):
    min_length = min(len(seq1), len(seq2))
    seq1 = seq1[:min_length]
    seq2 = seq2[:min_length]
    distance, _ = fastdtw(seq1, seq2)
    return distance

def calculate_trajectory_distance(trajectory):
    # Calculate Euclidean distance between consecutive points
    distances = np.linalg.norm(np.diff(trajectory, axis=0), axis=1)
    overall_distance = np.sum(distances)
    return overall_distance

def get_trajectory_files(shape, mode):
    folder_path = f"data/{mode}/{shape}/"
    return glob.glob(f"{folder_path}*.csv")

def parse_arguments():
    parser = argparse.ArgumentParser(description="Calculate DTW distances and overall measures for drone trajectories.")
    parser.add_argument("shape", choices=["ellipse", "lemniscate"], help="Shape of the trajectory (ellipse or lemniscate).")
    parser.add_argument("mode", choices=["autonomous", "piloted"], help="Mode of the trajectory (autonomous or piloted).")
    return parser.parse_args()

def main():
    args = parse_arguments()

    # Read trajectories from CSV files based on command-line arguments
    trajectory_files = get_trajectory_files(args.shape, args.mode)
    trajectories = [pd.read_csv(file)[['drone_x', 'drone_y', 'drone_z']].values for file in trajectory_files]

    # Calculate pairwise DTW distances
    pairwise_distances = np.zeros((len(trajectories), len(trajectories)))

    for i, j in combinations(range(len(trajectories)), 2):
        distance = calculate_dtw_distance(trajectories[i], trajectories[j])
        pairwise_distances[i, j] = distance
        pairwise_distances[j, i] = distance

    # Calculate overall distance traveled for each trajectory
    overall_distances = [calculate_trajectory_distance(trajectory) for trajectory in trajectories]

    # Calculate sum of distances flown
    sum_of_distances_flown = np.sum(overall_distances)

    # Print overall distances for each trajectory
    for i, distance in enumerate(overall_distances):
        print(f"Trajectory {i+1} Overall Distance: {distance} meters")

    # Print sum of distances flown
    print(f"Sum of Distances Flown: {sum_of_distances_flown} meters")

    # Calculate an overall measure (e.g., mean or sum) based on the pairwise distances
    overall_measure = np.mean(pairwise_distances)  # or np.sum(pairwise_distances)

    print(f"Overall Measure {args.shape.capitalize()} {args.mode.capitalize()}: {overall_measure}")

if __name__ == "__main__":
    main()
