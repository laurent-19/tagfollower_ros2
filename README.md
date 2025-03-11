# tagfollower_ros2

A ROS2 implementation of a drone tag tracking system that uses computer vision with Kalman filtering to detect and track visual tags.

## Description

This package provides functionality to track visual tags from a drone's camera feed. It:
- Subscribes to drone navigation data and camera images
- Processes images to detect tags
- Uses Kalman filtering for smooth tracking predictions
- Visualizes the tracking results

## Installation

### Prerequisites
- ROS2 Humble or later
- OpenCV 4.x with video module
- cv_bridge
- image_transport
- A compatible drone or recorded bag files

### Build Instructions
```bash
# Clone the repository into your ROS2 workspace
cd ~/your_workspace/src
git clone https://github.com/yourusername/tagfollower_ros2.git

# Build the package
cd ~/your_workspace
colcon build --packages-select tagfollower_ros2

# Source the workspace
source install/setup.bash

## Usage

### Running the Tag Follower Node
```bash
ros2 run tagfollower_ros2 tagfollower_node
```
### Bag Files

Sample ROS bag files for testing are available at:
- Test flight: https://drive.google.com/drive/folders/1A9WoOUehvs6-Td5MW19uodxnmYH1XlhD?usp=drive_link

### Converting ROS1 Bags to ROS2

To convert existing ROS1 bag files to ROS2 format:

1. Install the rosbags conversion tool:
```bash
pip3 install rosbags
```

2. Convert the bag file:
```bash
rosbags-convert --src ros1_bag.bag --dst ros2_bag_dir
```

This will create a new directory containing the converted ROS2 bag files.

Note: The converter maintains the original message types and topic names. Make sure your ROS2 workspace has all required message definitions.

### Important Note on Bag Conversion

When converting bags, you need to modify the metadata file:

1. Open `ros2_bag_dir/metadata.yaml`
2. Locate the `/ardrone/navdata` topic section
3. Update the message type:
```yaml
topic_metadata:
    name: /ardrone/navdata
    type: tagfollower_ros2/msg/Navdata
```

This step is required to ensure proper message type mapping in ROS2.


### Playing Recorded Data
```bash
# Navigate to your data directory
cd ~/your_workspace

# Play the bag file
ros2 bag play bags2
```
### Examining Recorded Data

```bash
# Get bag info
ros2 bag info bags2
# Output:
Files:             bags2.db3
Bag size:          147.2 MiB
Storage id:        sqlite3
Duration:          142.1s
Messages:          14268
Topic information: Topic: /camera/image_raw | Type: sensor_msgs/msg/Image
                  Topic: /drone/navdata   | Type: tagfollower_msgs/msg/Navdata

# List active topics
ros2 topic list
/ardrone/image_raw
/ardrone/navdata
/rosout
/tf

# Get topic info
ros2 topic info /camera/image_raw
Type: sensor_msgs/msg/Image

ros2 topic info /drone/navdata  
Type: tagfollower_msgs/msg/Navdata
```

## Message Types

This package defines several custom message types for drone navigation data:
- Navdata
- NavdataAltitude
- NavdataVisionDetect
- ...