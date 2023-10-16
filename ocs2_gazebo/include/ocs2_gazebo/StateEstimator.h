#pragma once

#include <functional>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>

#include "ocs2_gazebo/RobotState.h"
#include <std_msgs/Bool.h>

namespace gazebo {
class StateEstimator : public ModelPlugin {
   public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr sdf);
    void OnUpdate();

   private:
    // contact callback functions
    void lf_foot_contact_callback(const std_msgs::Bool::ConstPtr &msg) { contacts[0] = msg->data; }
    void rf_foot_contact_callback(const std_msgs::Bool::ConstPtr &msg) { contacts[1] = msg->data; }
    void lh_foot_contact_callback(const std_msgs::Bool::ConstPtr &msg) { contacts[2] = msg->data; }
    void rh_foot_contact_callback(const std_msgs::Bool::ConstPtr &msg) { contacts[3] = msg->data; }

    void loadSettings(const std::string &configFile);

    // callback function for Gazebo
    event::ConnectionPtr updateConnection;

    // robot state publisher
    ros::Publisher state_publisher;

    // robot in simulation
    physics::ModelPtr robot;

    // base link
    physics::LinkPtr base_link;

    // joints
    std::vector<physics::JointPtr> joints;

    // state publish rate
    double rate;

    // last time state message was publisher
    common::Time last_publish_time;

    // contacts
    std::vector<bool> contacts;
    ros::Subscriber lf_sub;
    ros::Subscriber lh_sub;
    ros::Subscriber rf_sub;
    ros::Subscriber rh_sub;

    // last yaw angle
    double yaw_last;
};
}  // namespace gazebo