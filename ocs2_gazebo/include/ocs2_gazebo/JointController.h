#pragma once

#include <unordered_map>
#include <utility>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <urdf/model.h>

#include "ocs2_gazebo/JointCommand.h"
#include "ocs2_gazebo/JointCommandArray.h"

namespace switched_model {

class JointController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
   public:
    // Destructor
    ~JointController();

    // initialize joint controller
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n);

    // update joint torques, this function gets called every simulation timestep
    void update(const ros::Time &time, const ros::Duration &period);

    // WBC controller command callback
    void command_callback(const ocs2_gazebo::JointCommandArrayConstPtr &command_ptr);

   private:
    // WBC controller command subscriber
    ros::Subscriber command_subscriber;

    // joint name -> JointHandle map
    std::unordered_map<std::string, hardware_interface::JointHandle> joint_map;

    // joint_name -> joint_limits map
    std::unordered_map<std::string, double> torque_limits;

    // command message realtime buffer
    typedef ocs2_gazebo::JointCommandArray BufferType;
    realtime_tools::RealtimeBuffer<BufferType> command_buffer;
};

}  // namespace switched_model
