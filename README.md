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

### Playing Recorded Data
```bash
# Navigate to your data directory
cd ~/your_workspace/src/tagfollower_ros2

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