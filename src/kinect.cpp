#include <ros/ros.h>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <klient/waveStep.h>
#include <ahrs_client/Data.h>

// typedef pcl::PointXYZ Point;
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::string nameOfFile = "CloudPoint";

void kinectCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(*input, *cloud);
	ROS_INFO("Point cloud saved.");
	nameOfFile += ".pcd";
	pcl::io::savePCDFileASCII(nameOfFile.c_str(), *cloud);
}

void ahrsDataCallback(const ahrs_client::Data input)
{
	ROS_INFO("AHRS - zAxisRotation: %f", input.zAxisRotation);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kinect", ros::init_options::AnonymousName);
	ros::NodeHandle nodeHandle;

	// Publisher
	ros::Subscriber kinectSubscriber = nodeHandle.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1000, kinectCallback);
	ros::Publisher waveStepPublisher = nodeHandle.advertise<klient::waveStep>("waveStep", 1000);
	ros::Subscriber ahrsDataSubscriber = nodeHandle.subscribe<ahrs_client::Data>("ahrs_data", 1000, ahrsDataCallback);

	// Message 
	klient::waveStep msg;
	msg.x = 0;
	msg.y = 0;
	msg.z = 0;
	msg.alpha = 0;
	msg.beta = 0;
	msg.gamma = 0.0;
	msg.speed = 0.1;
	msg.accel = 0.7;

	// Setting loop rate (Hz)
	ros::Rate loop_rate(0.2);
	int count = 0;
	while (ros::ok()) {
		if (count < 13) {
			msg.gamma = 0.05;
			waveStepPublisher.publish(msg);
			ROS_INFO("Gamma value: %f", msg.gamma);
			ros::spinOnce();
		}
		else {
			ROS_INFO("End of motion.");
		}
		loop_rate.sleep();
		++count;
	}
	return 0;
}