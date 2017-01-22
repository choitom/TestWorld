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
	
    
    // Rectangular region of interest
    int ROI_lo = height/3.95;	// 121
    int ROI_hi = height/1.12;	// 428
    
    Mat rect_roi(image.size(), image.type());
    image.copyTo(rect_roi);
    ROI(rect_roi, ROI_lo, ROI_hi);
    
    
    // Create sub ROIs
    int n_sub = 2;
    int interval = (ROI_hi-ROI_lo)/n_sub;
    
    int sub_lo = ROI_lo;
    int sub_hi;
    
    vector<Mat> subs;
    for(int i = 0; i < n_sub; i++)
    {
        Mat sub(rect_roi.size(), rect_roi.type());
        rect_roi.copyTo(sub);
        sub_hi = sub_lo + interval;
        ROI(sub, sub_lo, sub_hi);
        subs.push_back(sub);
        sub_lo = sub_hi;
    }
    
    
    // Process each sub ROI
    vector<Mat> proc_subs;
    for(size_t i = 0; i < subs.size(); i++)
    {
        proc_subs.push_back(preprocess(subs[i]));
    }
    
	
    // For each sub ROI, find vanishing point
    vector<Point> pts;
    for(size_t i = 0; i < proc_subs.size(); i++)
    {
        int lo = ROI_lo + i * interval;
        int hi = lo + interval;
        Point pt = vanishPoint(proc_subs[i], (lo+hi)/2);
        pts.push_back(pt);
        
        circle(image, pt, 2, Scalar(255,0,0), 3);
    }
    
    imshow("img", image);
    
    waitKey(4);
}

void CameraPlugin::ROI(Mat &mat, int lo, int hi)
{  
    for(size_t i = 0; i < mat.rows; i++)
    {
        if(i < lo || i > hi)
        {
            for(size_t j = 0; j < mat.cols; j++)
            {
                mat.at<Vec3b>(i,j)[0] = 0;
                mat.at<Vec3b>(i,j)[1] = 0;
                mat.at<Vec3b>(i,j)[2] = 0;
            }
        }
    }   
}

Point CameraPlugin::vanishPoint(Mat mat, int mid)
{
    vector<Vec2f> lines;
    HoughLines(mat, lines, 1, PI/180, 48, 0, 0);
    
    // inner most lines
    float rho_left = FLT_MAX, theta_left = FLT_MAX;
    float rho_right = FLT_MIN, theta_right = FLT_MIN;
    
    for(size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        if(0.1 < theta && theta < 1.5 || theta > 1.62 && theta < 3.14)
        {
            if(theta > PI/2)
            {
                if(theta-PI/2 > theta_right-PI/2)
                {
                    theta_right = theta;
                    rho_right = rho;
                }
            }
            if(theta < PI/2)
            {
                if(theta-PI/2 < theta_left-PI/2)
                {
                    theta_left = theta;
                    rho_left = rho;
                }
            }
        }
    }
    
    // convert polar to Cartesian
    double a1 = -(cos(theta_left)/sin(theta_left));
    double b1 = rho_left/sin(theta_left);
    
    double a2 = -(cos(theta_right)/sin(theta_right));
    double b2 = rho_right/sin(theta_right);
    
    // find intersection of two equations
    double x = (b1-b2)/(a2-a1);
    double y = a1 * x + b1;
    
    double x2 = mat.cols/2;
    double y2 = mat.rows;
    
    // y = mx + n, x = (y-n)/m
    double m = (y2-y)/(x2-x);
    double n = -m*x+y;
    
    // find x, given y = mid
    int waypoint_x = (mid-n)/m;
    
    return Point(waypoint_x,mid);
}

// Gray -> Gaussian Blur -> Morph Open -> Edge
Mat CameraPlugin::preprocess(Mat mat)
{
    Mat gray, morph, canny;
    cvtColor(mat, gray, CV_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5,5), 0, 0);
    
    Mat erosion(5, 5, CV_8U, Scalar(1));
    morphologyEx(gray, morph, MORPH_OPEN, erosion);
    
    Canny(gray, canny, 128, 255);
    
    return canny;
}