cmake_minimum_required (VERSION 2.8.3)
project (openni2_ros_wrapper)

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
	"~/OpenNI2/Include" 	# find include files for OpenNI2
)

# find libraries for OpenNI2
link_directories ("~/OpenNI2/Bin/Arm-Release")
link_directories ("~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers")
# add_library (OpenNI2 SHARED IMPORTED)
# set_target_properties (OpenNI2 PROPERTIES IMPORTED_LOCATION 
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libDummyDevice.so
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libOniFile.so
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libPS1080.so
# 	~/OpenNI2/Bin/Arm-Release/OpenNI2/Drivers/libPSLink.so
# )

add_executable (testDemo src/main.cpp include/OniSampleUtilities.h)
target_link_libraries (testDemo 
	${catkin_LIBRARIES} 
	# ${OpenCV_LIBRARIES}
	libDepthUtils.a libOpenNI2.so
	libDummyDevice.so libOniFile.so libPS1080.so libPSLink.so)
