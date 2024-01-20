import argparse
import pandas as pd
from fastdtw import fastdtw
import numpy as np
from itertools import combinations
import glob

def calculate_dtw_distance(seq1, seq2):
    # Function to calculate DTW distance between two trajectories
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

def get_trajectory_files(mode, shape):
    folder_path = f"../data/{mode}/*-{shape}/"
    print(f"Reading trajectories from {folder_path}")
    return glob.glob(f"{folder_path}*_500hz_freq_sync.csv")

def parse_arguments():
    parser = argparse.ArgumentParser(description="Calculate DTW distances and overall measures for drone trajectories.")
    parser.add_argument("--mode", choices=["autonomous", "piloted"], help="Mode of the trajectory (autonomous or piloted).")
    parser.add_argument("--shape", choices=["ellipse", "lemniscate", "trackRATM"], help="Shape of the trajectory (ellipse, lemniscate or trackRATM).")
    return parser.parse_args()

def main():
    args = parse_arguments()

    # Read trajectories from CSV files based on command-line arguments
    trajectory_files = get_trajectory_files(args.mode, args.shape)
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
        print(f"Trajectory {i+1} Distance Flown: {distance} meters")

    # Print sum of distances flown
    print(f"Sum of Distances Flown: {sum_of_distances_flown} meters")

    # Calculate an overall measure (e.g., mean or sum) based on the pairwise distances
    overall_measure = np.mean(pairwise_distances)  # or np.sum(pairwise_distances)

    print(f"Mean Dynamic Time Warping for {args.mode.capitalize()} {args.shape.capitalize()}: {overall_measure}")

if __name__ == "__main__":
    main()
