#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "CameraPlugin.hh"

using namespace gazebo;
using namespace cv;

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

// Pointer to the update event connection
event::ConnectionPtr updateConnection;
sensors::MultiCameraSensorPtr parentSensor;

void CameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
	this->parentSensor =
		boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(_sensor);

	if(!this->parentSensor)
	{
		gzerr << "Couldn't find a camera\n";
		return;
	}

	// connect to the sensor update event.
	this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&CameraPlugin::OnUpdate, this));

	// make sure the parent sensor is active
	this->parentSensor->SetActive(true);
}

void CameraPlugin::OnUpdate()
{
	// get image width, height, and data
	int width = (int) this->parentSensor->GetImageWidth(0);
	int height = (int) this->parentSensor->GetImageHeight(0);
	const unsigned char* imageData = this->parentSensor->GetImageData(0);

	// create image matrix
	Mat image = Mat(height, width, CV_8UC3, const_cast<unsigned char*>(imageData));
	

	// use Canny algorithm to apply on the image to get contours. (args: input, output, low threshold, high threshold)
  cv::Mat contours;
  cv::Canny(image,contours,125,350);

  // invert the image to better show the lines (threshold value below 128 becomes 255)
  cv::Mat contoursInv;
  cv::threshold(contours,contoursInv,128, 255, cv::THRESH_BINARY_INV);


	// camera view display
	namedWindow("Camera View", CV_WINDOW_AUTOSIZE);
	imshow("Camera View", contoursInv);
	waitKey(4);
}
