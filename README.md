# Race Against the Machine

> A Fully-annotated, Open-design Dataset of Autonomous and Piloted High-speed Flight

This repository contains a dataset characterized by:

- fast (>20m/s) and aggressive quadrotor flight
- autonomous and human-piloted flight, on multiple trajectories
- high-resolution, high-frequency collection of visual, inertial, and motion capture data
- it includes drone racing gates—with bounding boxes and individual corner labels
- it includes control inputs and battery voltages

If you use this repo, you can cite [the companion paper](https://arxiv.org/abs/2311.02667) as:

```bibtex
@misc{bosello2023race,
      title={Race Against the Machine: a Fully-annotated, Open-design Dataset of Autonomous and Piloted High-speed Flight}, 
      author={Michael Bosello and Davide Aguiari and Yvo Keuter and Enrico Pallotta and Sara Kiade and Gyordan Caminati and Flavio Pinzarrone and Junaid Halepota and Jacopo Panerati and Giovanni Pau},
      year={2023},
      eprint={2311.02667},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

## Installation/Download

Tested on October 2023 with **Ubuntu 20.04 LTS** (Python 3.8), **macOS 14** (Python 3.11), **Windows 11** (Python 3.9).

Clone the repository and install the requirements:

```sh
git clone https://github.com/Drone-Racing/drone-racing-dataset.git
cd drone-racing-dataset
pip3 install -r requirements.txt
```

From folder `drone-racing-dataset/`, use the following scripts to download the dataset files.

On **Ubuntu and macOS**:

```sh
sudo apt install wget   # or, e.g, `brew install wget` on macOS
sudo chmod +x data_downloader.sh
./data_downloader.sh
```

On **Windows**, double click on file [`drone-racing-dataset/data_downloader.cmd`](/data_downloader.cmd)

This will create and populate 2 folders in the root of the repository:

- `data/piloted/`
- `data/autonomous/`

Each of `data/piloted/` and `data/autonomous/` contains 12 `flight-.../` folders.

## Data Format

For each flight, 2 CSV files are provided:

- One sampled/interpolated at the timestamps of the camera frames (`..._cam_ts_sync.csv`)
- One sampled/interpolated at 500Hz (`..._500hz_freq_sync.csv`)

Each CSV file contains the following columns:

| Column Number and Quantity Name | Unit | Data Type |
| --- | --- | --- |
| 0. `elapsed_time` | $s$ | float |
| 1. `timestamp` | $\mu s$ | int |
| 2. `img_filename` | n/a | string |
| 3. `accel_[x/y/z]` | $m/s^2$ | float |
| 6. `gyro_[x/y/z]` | $rad/s$ | float |
| 9. `thrust[0-3]` | $1$ | float $\in [0,1]$ |
| 13. `channels_[roll/pitch/thrust/yaw]` | $1$ | int $\in [1000,2000]$ |
| 17. `aux[1-4]` | $1$ | int $\in [1000,2000]$ |
| 21. `vbat` | $V$ | float |
| 22. `drone_[x/y/z]` | $m$ | float |
| 25. `drone_[roll/pitch/yaw]` | $rad$ | float |
| 28. `drone_velocity_linear_[x/y/z]` | $m/s^2$ | float |
| 31. `drone_velocity_angular_[x/y/z]` | $rad/s$ | float |
| 34. `drone_residual` | $m$ | float |
| 35. `drone_rot[[0-8]]` | $1$ | float |
| 44. `gate[1-4]_int_[x/y/z]` | $m$ | float |
| 56. `gate[1-4]_int_[roll/pitch/yaw]` | $rad$ | float |
| 68. `gate[1-4]_int_residual` | $m$ | float |
| 72. `gate[1-4]_int_rot[[0-8]]` | $1$ | float |
| 108. `gate[1-4]_marker[1-4]_[x/y/z]` | $m$ | float |

## Image Format

For each flight, 2 folders contain the Arducam video capture data:

- `camera_flight-.../` contains all the captured frame in JPEG format
- `labels_flight-.../` contains the racing gates' bounding boxes and corner labels in the TXT format described below

### Labels Format

Each TXT file contains lines in the form `0 cx cy w h tlx tly tlv trx try trv brx bry brv blx bly blv` where:

- *0* is the class label for a gate (the only class in our dataset)
- *cx, cy , w, h ∈ [0, 1]* are a gate's bounding box center’s coordinates, width, and height, respectively
- *tlx, tly ∈ [0, 1], tlv ∈ [0; 2]* are the coordinates and visibility (0 outside the image boundaries; 2 inside the image boundaries) of the top-left internal corner. Similarly for *tr, bl, br*, the top-right, bottom-left, and bottom-right corners.

> All values are in pixel coordinates normalized with respect to image size. The keypoints label format follows the COCO definition. The gates/lines in each TXT file are not ordered.

## Visualization Scripts

The scripts in the [`scripts/`](/scripts/) folder can be used to visualize the data and to convert the data to other formats.

To plot one of the CSVs, for example, use:

```sh
cd scripts/
python3 ./data_plotting.py --csv-file ../data/autonomous/flight-01a-ellipse/flight-01a-ellipse_cam_ts_sync.csv
```

To visualize the Arducam frames and labels, use (press SPACE to advance, CTRL+C to exit), for example:

```sh
cd scripts/
python3 ./label_visualization.py --flight flight-01a-ellipse
```

## FPV Racing Drone Open Design

Folder [`quadrotor/`](/quadrotor/) contains the [bill of material](/quadrotor/bom.md) and [STL files](/quadrotor/3d_print/) of the COTS/open design of the racing drone used to collect the dataset.
A tutorial to assemble the drone is on [YouTube](https://youtu.be/xvOS7IEFxlU).

## ROS2 Bags

Tested on October 2023 with **Ubuntu 20.04 LTS**

ROS2 `.sqlite3` bags are stored in the `ros2bag_.../` folder of each flight.

The rosbags contain topic with custom messages defined in the repository [drone-racing-msgs](https://github.com/tii-racing/drone-racing-msgs).

To play a rosbag:

- Install ROS2 Humble following the [official guide](https://docs.ros.org/en/humble/Installation.html)
- Install the custom messages defined in [drone-racing-msgs](https://github.com/tii-racing/drone-racing-msgs)

```sh
mkdir -p ~/drone_racing_ws/src
cd ~/drone_racing_ws/src
git clone https://github.com/tii-racing/drone-racing-msgs.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

- Source the ROS2 workspace and play the rosbag

```sh
source ~/drone_racing_ws/install/setup.bash
ros2 bag play data/autonomous/flight-01a-ellipse/ros2bag_flight-01a-ellipse
```

- Print the topic list and echo a topic

```sh
source ~/drone_racing_ws/install/setup.bash
ros2 topic list
ros2 topic echo sensors/imu
```

## Additional Resources

```sh
drone-racing-dataset
├── camera_calibration
│   ├── calibration_results.json - Camera parameters in JSON format.
│   ├── calibration_results.npz - Camera parameters in NumPy format.
│   └── drone_to_camera.json - Translation from the drone center to the camera in JSON format.
├── ...
└── scripts
    ├── camera_calibration.py - Script used to generate the files in `camera_calibration/`.
    ├── create_std_bag.py - Script used to generate standard ROS2 bags with `Image`, `Imu`, and `PoseStamped` messages. 
    │                       Standard bags are not provided in the dataset because of their size (>10GB each).
    │                       You will need the ROS2 workspace with the custom messages installed (see section "ROS2 Bags").
    ├── data_interpolation.py - Script used to generate the comprehensive CSV files interpolated at arbitrary frequencies.
    ├── ...
    └── reference_controller.py - Python implementation of the PID controller used for the autonomous flights.
```

## License

The dataset and the code are released under the [MIT license](LICENSE).
