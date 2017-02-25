/*****************************************************************************
*                                                                            *
*  OpenNI 2.x Alpha                                                          *
*  Copyright (C) 2012 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  Licensed under the Apache License, Version 2.0 (the "License");           *
*  you may not use this file except in compliance with the License.          *
*  You may obtain a copy of the License at                                   *
*                                                                            *
*      http://www.apache.org/licenses/LICENSE-2.0                            *
*                                                                            *
*  Unless required by applicable law or agreed to in writing, software       *
*  distributed under the License is distributed on an "AS IS" BASIS,         *
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  *
*  See the License for the specific language governing permissions and       *
*  limitations under the License.                                            *
*                                                                            *
*****************************************************************************/
#include <stdio.h>
#include <sstream>

// ROS headers
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
/*
// TODO
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/PointCloud2.h>
*/

// OpenNI2 headers
#include <OpenNI.h>
#include "OniSampleUtilities.h"

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace std;
using namespace openni;

string node_name = "openni2_ros_wrapper";
string pub_topic_name = "/" + node_name + "/depth_raw";
int mode = 6;

int main(int argc, char **argv)
{
	/* Initialize ROS */
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh("~");
	ros::Publisher pub = nh.advertise<sensor_msgs::Image>(pub_topic_name, 10);	
	ros::Rate loop_rate(10);

	/* Initialize the Structure Sensor device */
	Status rc = OpenNI::initialize();
	if (rc != STATUS_OK)
	{
		ROS_ERROR("Initialize failed\n%s\n", OpenNI::getExtendedError());
		return 1;
	}

	Device device;
	rc = device.open(ANY_DEVICE);
	if (rc != STATUS_OK)
	{
		ROS_ERROR("Couldn't open device\n%s\n", OpenNI::getExtendedError());
		return 2;
	}

	VideoStream depth;

	const SensorInfo *sinfo = device.getSensorInfo(SENSOR_DEPTH);
	if (sinfo != NULL)
	{
		rc = depth.create(device, SENSOR_DEPTH);
		if (rc != STATUS_OK)
		{
			ROS_ERROR("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
			return 3;
		}

		
	}

	/* Set resolution and frame rate */
	const Array<VideoMode>& modesDepth = sinfo->getSupportedVideoModes();
	ROS_INFO("All supported video modes:");
	for (int i=0; i<modesDepth.getSize(); i++)
	{
		ROS_INFO("  %i: %i*%i, %i fps, %i format", i, modesDepth[i].getResolutionX(), 
			modesDepth[i].getResolutionY(), modesDepth[i].getFps(), modesDepth[i].getPixelFormat());
	}
	if (nh.hasParam("mode"))
		nh.getParam("mode", mode);
	rc = depth.setVideoMode(modesDepth[mode]);

	if (rc != STATUS_OK)
	{
		ROS_ERROR("Couldn't set resolution and fps\n%s\n", OpenNI::getExtendedError());
		return 4;
	}
	else
	{
		ROS_INFO("Set resolution and fps successful.");
		ROS_INFO("Resolution: %d-by-%d, fps: %d, format: %d", depth.getVideoMode().getResolutionY(), 
			depth.getVideoMode().getResolutionX(), depth.getVideoMode().getFps(), depth.getVideoMode().getPixelFormat());
	}
	
	/* Start the device */
	rc = depth.start();
	if (rc != STATUS_OK)
	{
		ROS_ERROR("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
		return 5;
	}

	
	VideoFrameRef frame;
	sensor_msgs::Image ros_depth;
	ros_depth.encoding = "32FC1"; // "mono16";
	ros_depth.is_bigendian = 0;
	unsigned char bytes[4];
	
	ROS_INFO("OpenNI2 is up and running.");
	// while (!wasKeyboardHit())
	while (ros::ok())
	{
		int changedStreamDummy;
		VideoStream* pStream = &depth;
		rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
		if (rc != STATUS_OK)
		{
			ROS_ERROR("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
			continue;
		}

		rc = depth.readFrame(&frame);
		if (rc != STATUS_OK)
		{
			ROS_ERROR("Read failed!\n%s\n", OpenNI::getExtendedError());
			continue;
		}

		if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
		{
			ROS_ERROR("Unexpected frame format\n");
			continue;
		}

	    DepthPixel* pDepth = (DepthPixel*)frame.getData();
	    int numPixels = frame.getHeight() * frame.getWidth();
		// int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;
		// ROS_INFO("[%08llu] %8d", (long long)frame.getTimestamp(), pDepth[middleIndex]);

		
		ros_depth.height = frame.getHeight();
		ros_depth.width = frame.getWidth();
		ros_depth.step = frame.getWidth() * 4;
		ros_depth.data.clear();
		for (int i=0; i<numPixels; i++)
		{
			/*
		    char hiByte = pDepth[i] >> 8;
		    char lowByte = pDepth[i] % 256;
		    ros_depth.data.push_back(hiByte);
		    ros_depth.data.push_back(lowByte);
		    */
		    float depthValueInMeters = (float)pDepth[i] / 1000;
		    memcpy(bytes, &depthValueInMeters, 4);
		    ros_depth.data.push_back(bytes[0]);
		    ros_depth.data.push_back(bytes[1]);
		    ros_depth.data.push_back(bytes[2]);
		    ros_depth.data.push_back(bytes[3]);
		    
		}
		// std::copy(pDepth, pDepth+numPixels, numPixels);
		pub.publish(ros_depth);
		// ROS_INFO("size: %8d", ros_depth.data.size() );
		
		ros::spinOnce();
		loop_rate.sleep();
	}

	depth.stop();
	depth.destroy();
	device.close();
	OpenNI::shutdown();

	return 0;
}
