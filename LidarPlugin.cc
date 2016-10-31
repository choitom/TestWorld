#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <vector>

#include "LidarPlugin.hh"

using namespace gazebo;

// register this plugin
GZ_REGISTER_SENSOR_PLUGIN(LidarPlugin)

// lidar
LidarPlugin::LidarPlugin() : SensorPlugin()
{
}

LidarPlugin::~LidarPlugin()
{
}

void LidarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Get the parent sensor.
    this->parentSensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
        gzerr << "Couldn't find a laser\n";
        return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(boost::bind(&LidarPlugin::OnUpdate, this));

    // Make sure the parent sensor is active.
    this->parentSensor->SetActive(true);
}

void LidarPlugin::OnUpdate()
{	
	// vector that holds distance for each beam
	std::vector<double>* rays = new std::vector<double>();
	for (unsigned int i = 0; i < this->parentSensor->GetRayCount(); i++){
    rays->push_back(this->parentSensor->GetRange(i));				
  }
	
	/* For each beam, print out the distance to an object.
		 If no object detect, prints out 'inf'
	std:: cout << "\nLidar Info\n";
	for(auto &i : *rays)
	{
		std:: cout << "(" << i << ") ";
	}*/
}
