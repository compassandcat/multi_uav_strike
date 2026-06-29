# Multi-UAV Strike System (ROS1)

A multi-rotor UAV strike system with gimbal-based target localization and guidance simulation.

## Key Features
- Multi UAV coordination with independent gimbal systems
- Particle filter-based target estimation
- Kinematic target motion simulation
- ROS1-based simulation framework

## Prerequisites
- Package requirement: [multiModeQuad_ROS](https://github.com/JJJJJJJack/multiModeQuad_ROS)

## Build Instructions
```bash
cd ~/catkin_ws
catkin_make --pkg multi_uav_strike
```

## Deployment on a New Board (e.g. rk3588)
Before building, install the required ROS dependencies and copy the MAVROS geographic lib data:

```bash
# ROS dependencies
apt-get install ros-noetic-tf
apt-get install ros-noetic-eigen-conversions
apt-get install ros-noetic-mavros*

# Ensure 'localhost' resolves (required by some ROS nodes)
echo "127.0.0.1 localhost" >> /etc/hosts

# Copy the egm96-5.pgm geoid file into the GeographicLib share dir
# (assumes the MAVROS environment runs inside a docker container named 'rk3588_ros')
docker cp egm96-5.pgm rk3588_ros:/usr/share/GeographicLib/geoids/egm96-5.pgm
```

Note: download the egm96-5.pgm beforehand from https://sourceforge.net/projects/geographiclib/files/geoids/egm96-5.pgm/download

## File Structure
```
├── launch/               # Simulation launch files
│   └── multi_uav_strike.launch
├── src/                  # Core implementation
│   ├── target_motion_simulator_node.cpp
│   ├── gimbal_simulator_node.cpp
│   ├── target_estimator_node.cpp
│   └── guidance_control_node.cpp
├── rviz/                 # Visualization config
│   └── view_strike.rviz
├── CMakeLists.txt        # Build configuration
├── package.xml           # ROS package metadata
└── README.md             # This document
```

## License
MIT License (See LICENSE file)

> Note: Guidance control node implementation is currently commented out in CMakeLists.txt - uncomment to enable.
