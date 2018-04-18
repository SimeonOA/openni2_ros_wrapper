# openni2_ros_wrapper
ROS wrapper for [OpenNI2](https://github.com/occipital/openni2), specifically for the Structure Sensor

## Installation
 - Install [OpenNI2](https://github.com/occipital/openni2): clone the OpenNI2 repo to `~/OpenNI2` and compile
 - Clone this repo under the source directory of your ROS workspace (e.g., under `~/catkin_ws/src/`)
 - Modify CMakeLists.txt. Specifically:
     - Line 5: if the OpenNI2 is not located at `~/OpenNI2`, modify the `OPENNI2_PATH` variable to your OpenNI2 directory
     - Line 34-35: modify the path, depending on the platform (`Arm-Release` or `x64-Release`)
 - Compile (e.g., run `catkin_make` under the directory `~/catkin_ws`)
 
## Usage
 - Run `source ~/catkin_ws/devel/setup.bash`
 - Launch the structure sensor with `roslaunch openni2_ros_wrapper structure_sensor.launch`

## Optional
To change the resolution and frame rate, modify the structure_sensor.launch file.
