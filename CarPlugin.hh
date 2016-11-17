#ifndef _CarPlugin_hh_
#define _CarPlugin_hh_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <string>
#include <vector>
#include "dataProcessing.hh"

namespace gazebo {
	class CarPlugin : public ModelPlugin {
		public:
			CarPlugin();
			void Load(physics::ModelPtr _model, sdf::ElementPtr _Sdf);
			void OnUpdate(const common::UpdateInfo & /*_info*/);
			void UpdateCarPosition();
		private:
			physics::ModelPtr model;
			event::ConnectionPtr updateConnection;
	};
}

#endif
