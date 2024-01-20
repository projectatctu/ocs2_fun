#include <ocs2_control/StateSubscriber.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

namespace ocs2_control {


/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
StateSubscriber::StateSubscriber(ros::NodeHandle &nh, const std::string &topic, const scalar_t initTime) {
    subscriber_ = nh.subscribe(topic, 1, &StateSubscriber::callback, this);
    stateReady_ = false;
    initTime_ = initTime;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
const SystemObservation &StateSubscriber::getObservation() {
    if (!stateReady_) {
        generateObservation();
    }
    return observation_;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
const SystemObservation &StateSubscriber::getObservation(const vector_t &grf) {
    if (!stateReady_) {
        generateObservation();
    }

    // Set ground reaction forces
    observation_.input.head<12>() = grf;

    return observation_;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StateSubscriber::waitTillInitialized() {
    while (!statePtr_) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();

        ROS_INFO_THROTTLE(1.0, "Waiting for state message...");
    }
    ROS_INFO("State message received.");
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StateSubscriber::generateObservation() {
    auto &robotState = *statePtr_;

    // Set observation time
    observation_.time = robotState.stamp.toSec() - initTime_;

    // Set mode
    std::array<bool, 4> contacts;
    std::copy(robotState.contacts.begin(), robotState.contacts.end(), contacts.begin());
    observation_.mode = switched_model::stanceLeg2ModeNumber(contacts);

    // Set state
    observation_.state = Eigen::Map<ocs2::vector_t>(robotState.x.data(), robotState.x.size());

    // Set input
    observation_.input.setZero(24);
    observation_.input.tail<12>() = Eigen::Map<ocs2::vector_t>(robotState.u.data(), robotState.u.size());

    stateReady_ = true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StateSubscriber::callback(const ocs2_gazebo::RobotState::Ptr &stateMsgPtr) {
    statePtr_ = stateMsgPtr;
    stateReady_ = false;
}

}  // namespace ocs2_control