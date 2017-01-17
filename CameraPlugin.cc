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
    Mat rect_roi = image.clone();
    int ROI_hi = (height*5/6);
    int ROI_lo = height/1.7;
    
    for(int i = 0; i < rect_roi.rows; i++)
    {
        if(i < ROI_lo || i > ROI_hi)
        {
            for(int j = 0; j < rect_roi.cols; j++)
            {
                rect_roi.at<Vec3b>(i,j)[0] = 0;
                rect_roi.at<Vec3b>(i,j)[1] = 0;
                rect_roi.at<Vec3b>(i,j)[2] = 0;
            }
        }
    }
    
    
    // Gray -> Gaussian blur -> Opening -> EDGE -> HOUGH
    Mat gray, canny, hough;
    
    cvtColor(rect_roi, gray, CV_BGR2GRAY);      // gray
    GaussianBlur(gray, gray, Size(5,5), 0, 0);  // blur
    
    Mat ero(5,5, CV_8U,Scalar(1));
    morphologyEx(gray, gray, MORPH_OPEN, ero);

    Canny(gray, canny, 128, 255);   // edge detect
    
    cvtColor(canny, hough, CV_GRAY2BGR);
    vector<Vec2f> lines;
    HoughLines(canny, lines, 1, PI/180, 50, 0, 0);  // hough
    
    // inner most lines
    float rho_left = FLT_MIN, theta_left = FLT_MIN;
    float rho_right = FLT_MIN, theta_right = FLT_MIN;
    
    for(size_t i = 0; i < lines.size(); i++)
    {
        float rho = lines[i][0], theta = lines[i][1];
        if(0.1 < theta && theta < 1.44 || theta > 1.66 && theta < 3.14)
        {
            Point pt1, pt2;
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            pt1.x = cvRound(x0 + 1000*(-b));
            pt1.y = cvRound(y0 + 1000*(a));
            pt2.x = cvRound(x0 - 1000*(-b));
            pt2.y = cvRound(y0 - 1000*(a));
            line(hough, pt1, pt2, Scalar(0,0,255), 2, CV_AA);
            
            // find the inner most lanes
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
                if(theta-PI/2 > theta_left-PI/2)
                {
                    theta_left = theta;
                    rho_left = rho;
                }
            }
        }   
    }
    
    // find intersection point of the innermost lines
    cout << theta_left << ", " << theta_right << endl;
    cout << rho_left << ", " << rho_right << endl;
    
    double a1 = -(cos(theta_left)/sin(theta_left));
    double b1 = rho_left/sin(theta_left);
    
    double a2 = -(cos(theta_right)/sin(theta_right));
    double b2 = rho_right/sin(theta_right);
    
    double x = (b1-b2)/(a2-a1);
    double y = a1 * x + b1;
    
    Point intersection = Point(x,y);
    
    // draw the intersection point
    circle(hough, intersection, 1, Scalar(255,0,0), 3);
    
    
    imshow("roi", rect_roi);
    imwrite("roi.bmp", rect_roi);
    
    imshow("canny", canny);
    imwrite("canny.bmp", canny);
    
    imshow("gray", gray);
    imwrite("gray.bmp", gray);
    
    imshow("hough", hough);
    imwrite("hough.bmp", hough);
    
    
	waitKey(4);
}
