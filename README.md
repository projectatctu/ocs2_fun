# ocs2_fun
ROS package for controlling Anymal and Spot in Gazebo using the OCS2 toolbox


# Installation
```bash

# Download ocs2 package
git clone https://github.com/projectatctu/ocs2.git

# Download ocs2_fun package
git clone https://github.com/projectatctu/ocs2_fun.git

# Download assets
git clone https://github.com/projectatctu/ocs2_robotic_assets.git

# Download pybind11 ros package
git clone https://github.com/wxmerkt/pybind11_catkin.git

# Download gridmap package
git clone https://github.com/ANYbotics/grid_map.git

# Download elevation mapping package
git clone https://github.com/ANYbotics/elevation_mapping.git

# Download elevation mapping cupy package
git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git

# Download kindr package
git clone https://github.com/ANYbotics/kindr.git

# Download kindr ros package
git clone https://github.com/ANYbotics/kindr_ros.git

# Download message logger package
git clone https://github.com/ANYbotics/message_logger.git

# Download qpoases catkin package
git clone https://github.com/wxmerkt/qpoases_catkin.git

# Set cmake build type
catkin config -DCMAKE_BUILD_TYPE=Release

# Build
catkin build

``````