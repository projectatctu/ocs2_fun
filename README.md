# ocs2_fun
ROS package for controlling Anymal and Spot in Gazebo using the OCS2 toolbox. Why? Because it's fun.

## Showcase


https://github.com/projectatctu/ocs2_fun/assets/82883398/ff3a9116-b92f-4512-b41e-e2358ca03f16


https://github.com/projectatctu/ocs2_fun/assets/82883398/96466319-4f5e-4866-869d-de34fbcdbb36




## Installation
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

## Worldgen
```bash
rosrun ocs2_gazebo world generate_world.py
```
