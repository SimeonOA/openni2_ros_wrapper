# openni2_ros_wrapper
ROS wrapper for [OpenNI2](https://github.com/occipital/openni2), specifically for the Structure Sensor

## Installation
 - Install [OpenNI2](https://github.com/occipital/openni2): clone the OpenNI2 repo to `~/OpenNI2` and compile
 - Clone this repo under the source directory of your ROS workspace (e.g., under `~/catkin_ws/src/`)
 - (Optional) Modify Line 5 in CMakeLists.txt and set the `OPENNI2_PATH` variable, if the OpenNI2 is not located at `~/OpenNI2`
 - Compile (e.g., run `catkin_make` under the directory `~/catkin_ws`)
 
## Usage
 - Run `source ~/catkin_ws/devel/setup.bash`
 - Launch the structure sensor with `roslaunch openni2_ros_wrapper structure_sensor.launch`

## Optional
To change the resolution and frame rate, modify the structure_sensor.launch file.
