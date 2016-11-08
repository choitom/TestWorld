#ifndef _dataProcessing_hh
#define _dataProcessing_hh

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <stdio.h>
#include <vector>
#include <map>

//#include "LidarInfo.hh"

namespace gazebo
{
	enum LidarPosition {FRONT, BACK};
	class dataProcessing
	{
		public:
			static void InitLidar(LidarPosition pos, double minAngle, double resolution, double maxRange, int numRays);
			static void UpdateCameraData(int newX, int newY);
			static void UpdateLidarData(LidarPosition pos, std::vector<double>* newData);
			static void GetLanePosition();			
			static std::vector<double>* GetLidarData(LidarPosition pos);
		private: 
			static int lanePositionX;
			static int lanePositionY;
			static std::vector<double>* frontLidarData;
			static std::vector<double>* backLidarData;
			//static std::map<LidarPosition, LidarInfo> lidarInfo;
	};
}
#endif
