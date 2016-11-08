#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <vector>
#include <map>
#include "dataProcessing.hh"

using namespace gazebo;

int dataProcessing::lanePositionX = 0;
int dataProcessing::lanePositionY = 0;
std::vector<double>* dataProcessing::frontLidarData = new std::vector<double>();
std::vector<double>* dataProcessing::backLidarData = new std::vector<double>();
std::map<LidarPosition, LidarInfo> dataProcessing::lidarInfo = std::map<LidarPosition, LidarInfo>();

// When initializing a lidar, store its information such as minimum angle, resoltuion and range
void dataProcessing::InitLidar(LidarPosition pos, double minAngle, double resolution, double maxRange, int numRays) {
	lidarInfo[pos] = LidarInfo(minAngle, resolution, maxRange, numRays);
}

// Update way point after image processing
void dataProcessing::UpdateCameraData(int newLanePositionX, int newLanePositionY) {
    lanePositionX = newLanePositionX;
    lanePositionY = newLanePositionY;
	std::cout << "Update(Camera)\n";	// check for update
}

// Update lidar data
void dataProcessing::UpdateLidarData(LidarPosition pos, std::vector<double>* newData) {
	switch (pos) {
		case FRONT:
			frontLidarData = newData;
		break;

		case BACK:
			backLidarData = newData;
		break;

		default:
		break;
	}
	std::cout << "Update(Lidar)\n";	// check for update
}

void dataProcessing::GetLanePosition() {
	
}

// Retrieve lidar data
std::vector<double>* dataProcessing::GetLidarData(LidarPosition pos) {
	switch (pos) {
		case FRONT:
			return frontLidarData;
		break;

		case BACK:
			return backLidarData;
		break;

		default:
		break;
	}
	return NULL;
}
