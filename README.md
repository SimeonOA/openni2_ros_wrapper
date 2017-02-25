# openni2_ros_wrapper
ROS wrapper for OpenNI2, specifically for the Structure Sensor

## Usage
 - Put the package under the source directory of your ROS workspace (e.g., under `~/catkin_ws/src/`)
 - Modify Line 5 in CMakeLists.txt to set the `OPENNI2_PATH` variable
 - Compile (e.g., run `catkin_make` under the directory `~/catkin_ws`)
 - Run `source ~/catkin_ws/devel/setup.bash`
 - Launch the structure sensor with `roslaunch openni2_ros_wrapper structure_sensor.launch`

## Optional
To change the resolution and frame rate, modify the structure_sensor.launch file.
