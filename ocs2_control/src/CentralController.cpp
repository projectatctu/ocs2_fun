#include <ocs2_control/CentralController.h>

namespace ocs2_control {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
CentralController::CentralController() {
    ros::NodeHandle nh;
    changeControllerSubscriber_ =
        nh.subscribe<std_msgs::String>("/change_controller", 1, &CentralController::changeControllerCallback, this);

    commandPublisher_ = nh.advertise<ocs2_gazebo::JointCommandArray>(
        "/anymal_joint_controller/joint_controller/switched_model/command", 1);

    initTime_ = ros::Time::now().toSec();
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void CentralController::visualize() { currentControllerPtr_->visualize(); }

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void CentralController::step(const scalar_t dt) {
    auto commandMessage = currentControllerPtr_->getCommandMessage(dt);
    commandPublisher_.publish(commandMessage);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void CentralController::addController(std::unique_ptr<Controller> controllerPtr) {
    controllerPtrs_.push_back(std::move(controllerPtr));

    if (controllerPtrs_.size() == 1) {
        currentControllerPtr_ = controllerPtrs_[0].get();
    }
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void CentralController::changeControllerCallback(const std_msgs::String::ConstPtr &msg) {
    const std::string controllerName = msg->data;
    for (size_t i = 0; i < controllerPtrs_.size(); ++i) {
        if (controllerPtrs_[i]->isSupported(controllerName)) {
            currentControllerPtr_ = controllerPtrs_[i].get();
            break;
        }
    }
    const scalar_t time = ros::Time::now().toSec() - initTime_;
    currentControllerPtr_->changeController(controllerName, time);
}

}  // namespace ocs2_control