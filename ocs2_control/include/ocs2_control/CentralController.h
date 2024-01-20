#pragma once

#include <ros/ros.h>
#include <memory>

#include <ocs2_control/Controller.h>
#include <ocs2_core/Types.h>

#include <std_msgs/String.h>

// Visualization

namespace ocs2_control {

using namespace ocs2;

class CentralController {
   public:
    CentralController();

    void step(const scalar_t dt);

    void visualize();

    void addController(std::unique_ptr<Controller> controllerPtr);

   private:

    // Change controller publisher
    ros::Subscriber changeControllerSubscriber_;
    void changeControllerCallback(const std_msgs::String::ConstPtr &msg);

    // Command publisher
    ros::Publisher commandPublisher_;

    Controller *currentControllerPtr_;

    std::vector<std::unique_ptr<Controller>> controllerPtrs_;

    scalar_t initTime_;
};

}  // namespace ocs2_control