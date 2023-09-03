import cv2
import numpy as np
import glob
from tqdm import tqdm
import argparse
import os
import json

CALIB_SHAPE = (6, 8)  # (rows, cols) as in released calibration images


def main(args):
    # Prepare object points
    obj_points = []  # 3D points in real-world space
    img_points = []  # 2D points in image plane

    # Generate object points
    objp = np.zeros((CALIB_SHAPE[0] * CALIB_SHAPE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CALIB_SHAPE[1], 0:CALIB_SHAPE[0]].T.reshape(-1, 2)

    # Read calibration images
    images = glob.glob(os.path.join(args.source, '*.jpg'))

    assert len(images) >= 10, "Too few images"
    print("Processing all images")
    for fname in tqdm(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (CALIB_SHAPE[1], CALIB_SHAPE[0]), None)

        # If corners are found, add object points and image points
        if ret:
            obj_points.append(objp)
            img_points.append(corners)
            if args.show:
                # Draw and display corners
                cv2.drawChessboardCorners(img, (CALIB_SHAPE[1], CALIB_SHAPE[0]), corners, ret)
                cv2.imshow('Chessboard Corners', img)
                cv2.waitKey(200)

    cv2.destroyAllWindows()
    print("Finding camera parameters")
    # Calibrate camera
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    # Print calibration results
    print("Camera Matrix:\n", mtx)
    print("Distortion Coefficients:\n", dist)

    # Save calibration results to a file
    print("Saving camera matrix and distortion coefficients into camera/calibration_results.npz")
    np.savez("camera/calibration_results.npz", mtx=mtx, dist=dist)

    print("Saving also in json file camera/calibration_results.json")
    with open("camera/calibration_results.json", "w") as f:
        output = {
            'mtx': np.array(mtx).tolist(),
            'dist': np.array(dist).tolist()
        }

        json.dump(output, f)

        f.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Camera calibration')
    parser.add_argument('--source', type=str, required=True,
                        help='Source folder with calibration images')
    parser.add_argument('--show', default=False, action="store_true",
                        help="Show target detection, do it only if you have few images.")
    args = parser.parse_args()

    main(args)
