cmake_minimum_required (VERSION 2.8.3)
project (openni2_ros_wrapper)

# Please set the OPENNI2_PATH variable to the root directory of your local, compiled OpenNI2 driver  
set(OPENNI2_PATH "~/OpenNI2")

# Get the information about this package's buildtime dependencies
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	genmsg
	message_generation 
	std_msgs 
	sensor_msgs
	# cv_bridge
	# image_transport
)

# Declare that this catkin package's runtime dependencies
catkin_package(CATKIN_DEPENDS 
	message_runtime 
	std_msgs 
	sensor_msgs
)


include_directories(include 
	${catkin_INCLUDE_DIRS}
	# ${OpenCV_INCLUDE_DIRS}
	"${OPENNI2_PATH}/Include" 	# find include files for OpenNI2
)

# find libraries for OpenNI2 on Arm
# link_directories ("${OPENNI2_PATH}/Bin/Arm-Release")
# link_directories ("${OPENNI2_PATH}/Bin/Arm-Release/OpenNI2/Drivers")
# find libraries for OpenNI2 on x64
link_directories ("${OPENNI2_PATH}/Bin/x64-Release")
link_directories ("${OPENNI2_PATH}/Bin/x64-Release/OpenNI2/Drivers")
# add_library (OpenNI2 SHARED IMPORTED)
# set_target_properties (OpenNI2 PROPERTIES IMPORTED_LOCATION 
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libDummyDevice.so
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libOniFile.so
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libPS1080.so
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libPSLink.so
# )

add_executable (structure_sensor src/main.cpp include/OniSampleUtilities.h)
target_link_libraries (structure_sensor 
	${catkin_LIBRARIES} 
	# ${OpenCV_LIBRARIES}
	libDepthUtils.a libOpenNI2.so
	libDummyDevice.so libOniFile.so libPS1080.so libPSLink.so)
