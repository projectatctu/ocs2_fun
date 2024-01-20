#pragma once

#include <string>
#include <ocs2_gazebo/RobotState.h>
#include <ocs2_mpc/SystemObservation.h>

#include <ros/ros.h>

namespace ocs2_control {

using namespace ocs2;

class StateSubscriber {
   public:
    StateSubscriber(ros::NodeHandle &nh, const std::string &topic, const scalar_t initTime);
    const SystemObservation &getObservation();
    const SystemObservation &getObservation(const vector_t &grf);
    void waitTillInitialized();

   private:
    void generateObservation();
    void callback(const ocs2_gazebo::RobotState::Ptr &stateMsgPtr);
    ros::Subscriber subscriber_;

    ocs2_gazebo::RobotState::Ptr statePtr_;
    bool stateReady_;
    SystemObservation observation_;

    scalar_t initTime_;
};

}  // namespace ocs2_control