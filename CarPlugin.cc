#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include "CarPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(CarPlugin)

// constructor
CarPlugin::CarPlugin() {

}

// load the car model
void CarPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) {
	std::cout << "Model plugin connected\n";
	// store the pointer to the model
	this->model = _model;

	// listen to the update event
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&CarPlugin::OnUpdate, this, _1));
}

// called by the world to update the model
void CarPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {
	dataProcessing::GetLanePosition();
	std::vector<double>* frontLidar = dataProcessing::GetLidarData(FRONT);

	// apply small linear velocity for testing
	this->model->SetLinearVel(math::Vector3(3,0,0));
}
