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

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

#include "CameraPlugin.hh"

#define PI 3.14159265359

using namespace gazebo;
using namespace cv;
using namespace std;

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
	// get image width, height and data
	int width = (int)this->parentSensor->GetImageWidth(0);
	int height = (int)this->parentSensor->GetImageHeight(0);
	const unsigned char* imageData = this->parentSensor->GetImageData(0);

	// create image matrix
	Mat image = Mat(height, width, CV_8UC3, const_cast<unsigned char*>(imageData));
	Mat imageBirdsEye = Mat(height, width, CV_8UC3, const_cast<unsigned char*>(imageData));
	

	// use Canny algorithm to apply on the image to get contours. (args: input, output, low threshold, high threshold)
	Mat contours;
	Canny(image,contours,125,350);

	// invert the image to better show the lines (threshold value below 128 becomes 255)
	Mat contoursInv;
	threshold(contours,contoursInv,128,255,THRESH_BINARY_INV);

	// Hough transform for line detection
	//float PI = 3.141592;
	std::vector<Vec2f> lines;
	HoughLines(contours,lines,1,PI/180, 75);

	// // Update our steering based on the lane markers
	// double angle_average = (left_lane_marker[1]+right_lane_marker[1]) /2;
	// double newSteeringAmount = -20*PI*(fabs(angle_average - PI/2))+50;
	// sdcSensorData::UpdateSteeringMagnitude(newSteeringAmount);

	//////////////// Perspective Transform ////////////////////
	///// adapted from http://stackoverflow.com/questions/7838487/executing-cvwarpperspective-for-a-fake-deskewing-on-a-set-of-cvpoint ///////

	//find actual points based on hough lines? for now these are hardcoded
    vector<Point> trapezoid_view_points;
    trapezoid_view_points.push_back(Point((width-1)/3, (height-1)/2)); 
    trapezoid_view_points.push_back(Point(((width-1)/3)*2, (height-1)/2));
    trapezoid_view_points.push_back(Point(width-1, ((height-1)/3)*2));
    trapezoid_view_points.push_back(Point(0, ((height-1)/3)*2));


    const Point* point = &trapezoid_view_points[0];
    int n = (int)trapezoid_view_points.size();
    Mat draw = contoursInv.clone();
    polylines(draw, &point, &n, 1, true, Scalar(0, 0, 0), 3, CV_AA); //CV_AA is anti-aliasing flag
    // save this for debugging
    imwrite("draw.jpg", draw);

    // Assemble a rotated rectangle out of that info
    RotatedRect box = minAreaRect(cv::Mat(trapezoid_view_points));
    std::cout << "Rotated box set to (" << box.boundingRect().x << "," << box.boundingRect().y << ") " << box.size.width << "x" << box.size.height << std::endl;

    Point2f pts[4];

    box.points(pts);

    cv::Point2f src_vertices[3];
    src_vertices[0] = pts[0];
    src_vertices[1] = pts[1];
    src_vertices[2] = pts[3];
    //src_vertices[3] = trapezoid_view_points[3];

    Point2f dst_vertices[3];
    dst_vertices[0] = Point(0, 0);
    dst_vertices[1] = Point(box.boundingRect().width-1, 0); 
    dst_vertices[2] = Point(0, box.boundingRect().height-1);

   /* Mat warpMatrix = getPerspectiveTransform(src_vertices, dst_vertices);

    cv::Mat rotated;
    cv::Size size(box.boundingRect().width, box.boundingRect().height);
    warpPerspective(src, rotated, warpMatrix, size, INTER_LINEAR, BORDER_CONSTANT);*/
    Mat warpAffineMatrix = getAffineTransform(src_vertices, dst_vertices);

    cv::Mat rotated;
    cv::Size size(box.boundingRect().width, box.boundingRect().height);
    warpAffine(contoursInv, rotated, warpAffineMatrix, size, INTER_LINEAR, BORDER_CONSTANT);

    cv::transpose(rotated, rotated);
    cv::flip(rotated, rotated, 0);

    imwrite("rotated.jpg", rotated);
    //namedWindow("Camera View", CV_WINDOW_AUTOSIZE);
    //imshow("CameraView", rotated);
	/////////////// end perspective transform //////////////

	/////////// feature detection /////////////////////
    ////// from http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_detection/feature_detection.html  ///////////

	int minHessian = 1200;

	SurfFeatureDetector detector( minHessian );

	//TODO: iterate through vector, make vector of limited number of points per y window 
	// (get using average)

	std::vector<KeyPoint> keypoints_1; //, keypoints_2;

	int waypointChunks = 6; // number of waypoints
	std::vector<KeyPoint> waypointPoints;

	// std::cout << "Keypoint x and y: (" << keypoints_1[0].pt << "," << keypoints_1[0].pt << ") " << std::endl;// << box.size.width << "x" << box.size.height << std::endl;


	//for (int i = 0; i < waypointChunks; i++) {
		//std::cout << "Keypoint x and y: (" << keypoints_1[i].pt.x << "," << keypoints_1[i].pt.y << ") " << std:: endl;// << box.size.width << "x" << box.size.height << std::endl;
		// printf("keypoint x: %f \n", keypoints_1[i].pt.x);

		// printf("keypoint y: %f \n", keypoints_1[i].pt.y);
		//for 
	//}



	detector.detect( contoursInv, keypoints_1 );
	//detector.detect( img_2, keypoints_2 );

	//-- Draw keypoints
	Mat img_keypoints_1; 
	//Mat img_keypoints_2;

	drawKeypoints( contoursInv, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
	//drawKeypoints( img_2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );

	//-- Show detected (drawn) keypoints
	imshow("Keypoints 1", img_keypoints_1 );
	//imshow("Keypoints 2", img_keypoints_2 );



	//////////// end feature detection /////////////////


	// camera view display
	//namedWindow("Camera View", CV_WINDOW_AUTOSIZE);
	//imshow("Camera View", contoursInv);
	
	// update way point data	
	dataProcessing::UpdateCameraData(0, 0);	

	// camera display
	// imshow("Contour Inverse", contoursInv);
	// imshow("Contour", contours);
	waitKey(4);
}
