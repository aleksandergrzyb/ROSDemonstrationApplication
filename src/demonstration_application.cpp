#include <ros/ros.h>
#include <sstream>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <klient/movePlatform.h>
#include <ahrs_client/Data.h>

pcl::PointCloud<pcl::PointXYZRGB> cloud;
bool dataSaved;

void kinectCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::fromROSMsg(*input, cloud);
	dataSaved = true;
	ROS_INFO("Point cloud received.");
}

void ahrsDataCallback(const ahrs_client::Data input)
{
	ROS_INFO("AHRS - xAxisRotation: %f", input.xAxisRotation);
	ROS_INFO("AHRS - yAxisRotation: %f", input.yAxisRotation);
	ROS_INFO("AHRS - zAxisRotation: %f", input.zAxisRotation);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demonstration_application", ros::init_options::AnonymousName);
	ros::NodeHandle nodeHandle;
	dataSaved = false;
	// Publisher
	ros::Publisher movePlatformPublisher = nodeHandle.advertise<klient::movePlatform>("movePlatform", 1);
	ros::Subscriber kinectSubscriber = nodeHandle.subscribe<sensor_msgs::PointCloud2>("camera/depth_registered/points", 1, kinectCallback);
	ros::Subscriber ahrsDataSubscriber = nodeHandle.subscribe<ahrs_client::Data>("ahrs_data", 1, ahrsDataCallback);

	// Message 
	klient::movePlatform msg;
	msg.x = 0;
	msg.y = 0;
	msg.z = 0;

	msg.alpha = 0;
	msg.beta = 0;
	msg.gamma = 0;

	msg.speed = 0.1;
	msg.accel = 0.7;

	// Needed for name of point cloud file
	std::string nameOfFile;

	// Setting loop rate (Hz)
	ros::Rate loop_rate(0.4);
	int incrementGamma = 0;
	int incrementAlpha = 0;
	int count = 0;

	std::stringstream stringStream;

	while (ros::ok()) {
		
		ros::spinOnce();
		stringStream.str("");
		stringStream << "PointCloud" << count << ".pcd";
		nameOfFile = stringStream.str();
		if (dataSaved) {
			pcl::io::savePCDFileASCII(nameOfFile.c_str(), cloud);
			ROS_INFO("Point Cloud saved.");
			msg.alpha = 0.3;
			msg.gamma = 0.3;
		}

		movePlatformPublisher.publish(msg);
		
		// if (!robotMoved) {
			// msg.gamma += incrementGamma;
			// msg.alpha += incrementAlpha;

			// incrementGamma += 0.05;
			// incrementAlpha += 0.01;
			
			// movePlatformRobotPublisher.publish(msg);
			// ros::spinOnce();
		// 	robotMoved = true;
		// }
		// else {
		// 	dataSaved = false;
		// 	ROS_INFO("End of motion.");
		// }
		
		loop_rate.sleep();
		++count;
	}
	return 0;
}