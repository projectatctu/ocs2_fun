# ocs2_fun
ROS package for controlling Anymal and Spot in Gazebo using the OCS2 toolbox


# Installation
```bash

# Clone this repository
git clone --recursive git@github.com:projectatctu/ocs2_fun.git

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Set build type - we recommend Release mode
catkin config -DCMAKE_BUILD_TYPE=Release

# Build
catkin build
```

# Worldgen
```bash
rosrun ocs2_gazebo world generate_world.py
```