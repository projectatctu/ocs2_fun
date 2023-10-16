#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>

#include "std_msgs/Bool.h"

namespace gazebo
{
  class ContactSensor : public SensorPlugin
  {
    public: 
    	ContactSensor() : SensorPlugin() {};
		void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf);
		void OnUpdate();

    private: 
      void loadSettings(const std::string &configFile);

    	sensors::ContactSensorPtr parentSensor;
		  event::ConnectionPtr updateConnection;
		  ros::Publisher contact_publisher;
      std_msgs::Bool contact_msg_;
      bool last_state;
  };
}