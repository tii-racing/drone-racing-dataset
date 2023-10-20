# drone-racing-dataset
Race Against the Machine: a Fully-annotated, Open-design
Dataset of Autonomous and Piloted High-speed Flight

## Paper
If you use this repo, please cite our paper:

*Race Against the Machine: a Fully-annotated, Open-design
Dataset of Autonomous and Piloted High-speed Flight* [[DOI]](link)]

```
@Article{
    TODO
}
```
## Introduction
Dataset for drone racing that is characterized by:
- fast (>20m/s), aggressive flight
- autonomous and human-piloted flights, on multiple trajectories
- high-resolution, high-frequency collection of visual, inertial, and motion capture data
- presence of drone racing gates fully labeled to the level of individual corners
- includes commands, inputs, and battery voltages


### Data Preview

Columns of the comprehensive synchronized CSVs:

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

### Drone Design

The directory *quadrotor* contains the bill of material and STL files of the open design (with commercial off-the-shelf components) of the racing drone used to collect the data. In the BOM it's present a link to the tutorial to assemble the drone.

## Data Usage
Data are provided in both CSV and ROS2 format.

Download the dataset from the [release section](https://github.com/Drone-Racing/drone-racing-dataset/releases)

    mkdir data && cd data
    wget LINK_TO_RELEASE

Unzip the downloaded files and then unzip the images and labels of each flight:

    find . -type f -name '*.zip' -exec unzip {} \; #unzip autonomous and piloted
    find . -type f -name '*.zip' -exec unzip {} \; #unzip images and labels

### Images and Labels usage
Each flight contains a folder with the images and a folder with the labels. The images are stored with path `{FLIGHT_NAME}/camera_{FLIGHT_NAME}/{IMG_NUM}_{TIMESTAMP}.jpg` and the labels with path `{FLIGHT_NAME}/labels_{FLIGHT_NAME}/{IMG_NUM}_{TIMESTAMP}.txt`. Each line in a TXT file represents a single gate in the form:
`0 cx cy w h tlx tly tlv trx try trv brx bry brv blx bly blv` where _0_ is the class label for a gate (the only class in
our dataset); _cx, cy , w, h ∈ [0, 1]_ are its bounding box center’s coordinates, width, and height, respectively; and _tlx, tly ∈ [0, 1], tlv ∈ [0; 2]_ are the coordinates and visibility (0 outside the image boundaries; 2 inside the image boundaries) of the top-left internal corner. Similarly for _tr, bl, br,_ the top-right, bottom-left, and bottom-right corners. All values are in pixel coordinates normalized with respect to image size. The keypoints label format follows the COCO definition.

### CSVs usage
The easiest way to use the data is to use one of the two comprehensive files provided in the root of each flight (which has been created using the script _data_interpolation.py_), which contains the synchronized data from all the sources.
In each file ending with `cam_ts_sync.csv`, the timestamps are from the camera frames, and all other data points are linearly interpolated to these timestamps. In each file ending with `500hz_freq_sync.csv`, timestamps are sampled at 500Hz frequency between the first and last camera timestamps and all other data points are linearly interpolated. In this case, each row references the file name of the camera frame with the closest timestamp.

If you want to use the raw data (which are not synchronized), you can use the CSVs in the `csv_raw` folder. Each CSV contains the data from a single source.

### ROS2 bags play
The ROS2 bags are stored in the `ros2bag_{FLIGHT_NAME}` folder of each file. The rosbag is in sqlite3 format. A dump of the data is available in CSV format in the `csv_raw/ros2bag_{FLIGHT_NAME}` folder. The CSVs are named after the topic name.
The rosbag contains topic with custom messages defined in the repository [drone-racing-msgs](https://github.com/Drone-Racing/drone-racing-msgs).

To play the rosbag you need to:

#### 1) Install ROS2 Humble following the [official guide](https://docs.ros.org/en/humble/Installation.html).
#### 2) Install the custom messages defined in the repository [drone-racing-msgs](https://github.com/Drone-Racing/drone-racing-msgs).

    mkdir -p ~/drone_racing_ws/src
    cd ~/drone_racing_ws/src
    git clone https://github.com/Drone-Racing/drone-racing-msgs.git
    cd ..
    colcon build --symlink-install

#### 3) Source the ROS2 workspace and Play the rosbag

    source ~/drone_racing_ws/install/setup.bash
    ros2 bag play data/autonomous/flight-01a-ellipse/ros2bag_flight-01a-ellipse

## Scripts Quick Start

### 1) Download the repository

    git clone https://github.com/Drone-Racing/drone-racing-dataset.git
    cd drone-racing-dataset
    git submodule update --init

### 2) Install requirements

    pip install -r requirements.txt

### 3) Download the datasets

Download the dataset from the [release section](https://github.com/Drone-Racing/drone-racing-dataset/releases) and put the zip files in the directory `data`.

    cd data
    wget LINK_TO_RELEASE

Unzip the downloaded files and then unzip the images and labels of each flight:

    find . -type f -name '*.zip' -exec unzip {} \; #unzip autonomous and piloted
    find . -type f -name '*.zip' -exec unzip {} \; #unzip images and labels

### 4) Run one of the scripts

    cd ../scripts

#### 4.1) Data interpolation

    python3 data_interpolation.py --flight flight-01p-ellipse

#### 4.2) Data visualization 
Available subplots: '3d', 'accel', 'gyro', 'thrust', 'vbat', 'position', 'rotation', 'channels'.
Default: all.

    python3 data_plotting.py --csv-file flight-01p-ellipse_cam_ts_sync.csv --subplots 3d accel gyro

#### 4.3) Label visualization

    python3 label_visualization.py --flight flight-01a-ellipse

#### 4.4) Standard bag creation

    python3 standard_bag_conversion.py --flight flight-01a-ellipse

## Resources
- *camera_calibration*:
    - *calibration_results.json* Camera parameters of the camera used in the dataset in json format.
    - *calibration_results.npz* Camera parameters of the camera used in the dataset in numpy format.
    - *drone_to_camera.json* Translation from the drone center to the camera in json format.
- *drone_racing_msgs*: Custom ROS2 messages used in the dataset.
- *quadrotor*: Bill of materials and STL files of the quadrotor used in the dataset.
- *scripts*: Python scripts to visualize and convert the data.
    - *reference_controller.py*: Python code that mimic the reference controller used for the autonomous control of the quadrotor.
    - *camera_calibration.py*: Script used to generate the files under camera_calibration.

## License
The dataset and the code are released under the [MIT license](LICENSE).