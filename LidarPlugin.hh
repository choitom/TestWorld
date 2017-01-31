#ifndef _GAZEBO_LIDAR_PLUGIN_HH_
#define _GAZEBO_LIDAR_PLUGIN_HH_

#include <stdio.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <vector>
#include <math.h>

#include "dataProcessing.hh"

namespace gazebo
{
	class LidarPlugin : public SensorPlugin
	{
		public: LidarPlugin();
		public: virtual ~LidarPlugin();

		public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/);
		private: void OnUpdate();
		private: void getVisibleObject(std::vector<double>* objectRays);
		
		private: sensors::RaySensorPtr parentSensor;
		private: event::ConnectionPtr updateConnection;
	};
}
#endif
