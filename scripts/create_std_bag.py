import os
import re
import sys
import zipfile
import pandas as pd
import cv2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py

def get_rosbag_options(path, storage_id, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(
        uri=path, storage_id=storage_id)

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def convert_to_nanosec(timestamp_us):
    return timestamp_us * 1000

def create_imu_msg(timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
    assert len(str(timestamp)) == 19
    imu_msg = Imu()
    imu_msg.header.stamp.sec = timestamp // 1000000000
    imu_msg.header.stamp.nanosec = timestamp % 1000000000
    imu_msg.linear_acceleration.x = accel_x
    imu_msg.linear_acceleration.y = accel_y
    imu_msg.linear_acceleration.z = accel_z
    imu_msg.angular_velocity.x = gyro_x
    imu_msg.angular_velocity.y = gyro_y
    imu_msg.angular_velocity.z = gyro_z
    return imu_msg

def create_pose_msg(timestamp, pose):
    assert len(str(timestamp)) == 19
    pose_img = PoseStamped()
    pose_img.header.stamp.sec = timestamp // 1000000000
    pose_img.header.stamp.nanosec = timestamp % 1000000000
    pose_img.pose = pose
    return pose_img

def create_image_msg(image_path, timestamp):
    assert len(str(timestamp)) == 19
    image = cv2.imread(image_path)
    bridge = CvBridge()
    image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")
    image_msg.header.stamp.sec = timestamp // 1000000000
    image_msg.header.stamp.nanosec = timestamp % 1000000000
    return image_msg

def create_topic(writer, topic_name, topic_type, serialization_format='cdr'):
    topic_name = topic_name
    topic = rosbag2_py.TopicMetadata(name=topic_name, type=topic_type,
                                     serialization_format=serialization_format)

    writer.create_topic(topic)

def validate_folder_name(folder_name):
    pattern = r'^flight-\d{2}[ap]-(?:ellipse|lemniscate)$'
    return re.match(pattern, folder_name)

def main():
    if len(sys.argv) != 2:
        print("Usage: python create_std_bag.py <flight_path>")
        return

    flight_run_path = sys.argv[1]

    if validate_folder_name(flight_run_path):
        folder_name = flight_run_path
        bag_path = os.path.join(folder_name, f"ros2bag_{folder_name}")
    else:
        print("Invalid folder name format (flight-XXy-[ellipse|lemniscate])")
        sys.exit(1)

    imu_df = pd.DataFrame(columns=["timestamp", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"])
    qualisys_df = pd.DataFrame(columns=["timestamp", "pose", "twist"])
    image_paths = []

    # Read IMU data from rosbag
    storage_options, converter_options = get_rosbag_options(bag_path, 'sqlite3')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=['/sensors/imu', '/perception/drone_state'])
    reader.set_filter(storage_filter)

    imu_data_list = []
    qualisys_data_list = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        if(msg_type.__name__ == 'SensorImu'):
            imu_data = deserialize_message(data, msg_type)
            imu_data_list.append({
                "timestamp": imu_data.timestamp,
                "accel_x": imu_data.accel_x,
                "accel_y": imu_data.accel_y,
                "accel_z": imu_data.accel_z,
                "gyro_x": imu_data.gyro_x,
                "gyro_y": imu_data.gyro_y,
                "gyro_z": imu_data.gyro_z,
            })
        if(msg_type.__name__ == 'DroneState'):
            qualisys_data = deserialize_message(data, msg_type)
            qualisys_data_list.append({
                "timestamp": qualisys_data.timestamp,
                "pose": qualisys_data.pose,
                "velocity": qualisys_data.velocity,
            })

    
    del reader

    imu_df = pd.DataFrame(imu_data_list)
    qualisys_df = pd.DataFrame(qualisys_data_list)

    # Extract image paths from the zipfile
    zip_path = os.path.join(folder_name, f"camera_{folder_name}.zip")
        
    with zipfile.ZipFile(zip_path, "r") as zip_ref:
        image_paths = [file for file in zip_ref.namelist() if file.lower().endswith(('.jpg', '.jpeg', '.png', '.gif'))]

        for path in image_paths:
            zip_ref.extract(path, flight_run_path)

    # Create a ROS2 bag for output
    output_bag_path = os.path.join(flight_run_path, "imu_cam_bag")
    storage_options, converter_options = get_rosbag_options(output_bag_path, 'sqlite3')
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

    # create topics
    create_topic(writer, '/sensors/imu', 'sensor_msgs/msg/Imu')
    create_topic(writer, '/camera/image', 'sensor_msgs/msg/Image')
    create_topic(writer, '/perception/drone_state', 'geometry_msgs/PoseStamped')

    start_time = imu_df["timestamp"].min()
    end_time = imu_df["timestamp"].max()

    print("Converting IMU...")
    for _, row in imu_df.iterrows():
        imu_msg = create_imu_msg(
            convert_to_nanosec(int(row["timestamp"])),
            row["accel_x"], row["accel_y"], row["accel_z"],
            row["gyro_x"], row["gyro_y"], row["gyro_z"]
        )
        #print(imu_msg.header.stamp.sec, imu_msg.header.stamp.nanosec, 'IMU')
        writer.write('/sensors/imu', serialize_message(imu_msg), int(convert_to_nanosec(row["timestamp"])))
    
    print("Converting POSE...")
    for _, row in qualisys_df.iterrows():
        pose_msg = create_pose_msg(
            convert_to_nanosec(int(row["timestamp"])),
            row["pose"]
        )
        #print(pose_msg.header.stamp.sec, pose_msg.header.stamp.nanosec, 'POSE')
        writer.write('/perception/drone_state', serialize_message(pose_msg), int(convert_to_nanosec(row["timestamp"])))
    
    print("Converting IMAGE...(it may take several GBs)")
    for image_path in image_paths[1:]:
        filename = image_path.split('/')[-1]
        timestamp = filename.split('_')[1].split('.')[0]
        image_msg = create_image_msg(
            os.path.join(flight_run_path, image_path),
            convert_to_nanosec(int(timestamp))
        )
        #print(image_msg.header.stamp.sec, image_msg.header.stamp.nanosec, 'IMAGE')
        writer.write('/camera/image', serialize_message(image_msg), convert_to_nanosec(int(timestamp)))

    del writer

    duration = end_time - start_time

if __name__ == "__main__":
    main()
