#include <ocs2_control/WbcController.h>

#include "ocs2_anymal_mpc/AnymalInterface.h"
#include <ocs2_core/misc/LoadData.h>

namespace ocs2_control {
WbcController::WbcController(const std::string &configFile, const std::string &frameDeclarationFile,
                             const std::string &taskSettingsFile, const std::string &controllerConfigFile,
                             const std::string &urdfString, switched_model::SqpWbc &sqp,
                             std::shared_ptr<switched_model::QuadrupedVisualizer> visualizerPtr,
                             std::shared_ptr<StateSubscriber> stateSubscriberPtr)
    : sqpWbc_(sqp), visualizerPtr_(visualizerPtr), mrt_("anymal") {
    stateSubscriberPtr_ = stateSubscriberPtr;

    ros::NodeHandle nh;
    mrt_.launchNodes(nh);
    tNow_ = 0.0;

    // Load mpc rate
    ocs2::loadData::loadCppDataType<scalar_t>(taskSettingsFile, "mpc.mpcDesiredFrequency", mpcRate_);
}

void WbcController::visualize() {
    if (timeSinceLastMpcUpdate_ == 0.0) {
        visualizerPtr_->update(stateSubscriberPtr_->getObservation(), mrt_.getPolicy(), mrt_.getCommand());
    }
}

void WbcController::changeController(const std::string &controllerType, const scalar_t time) {
    if (!mrt_initialized_ || time + 0.1 > mrt_.getPolicy().timeTrajectory_.back()) {
        resetMpc();
        mrt_initialized_ = true;
    }
    tNow_ = time;
}

void WbcController::resetMpc() {
    // Generate initial observation
    stateSubscriberPtr_->waitTillInitialized();
    auto initialObservation = stateSubscriberPtr_->getObservation();
    const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initialObservation.state},
                                                          {initialObservation.input});
    mrt_.resetMpcNode(initTargetTrajectories);

    while (!mrt_.initialPolicyReceived() && ros::ok()) {
        ROS_INFO("Waiting for initial policy...");
        ros::spinOnce();
        mrt_.spinMRT();
        initialObservation = stateSubscriberPtr_->getObservation();
        mrt_.setCurrentObservation(initialObservation);
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Initial policy received.");
}

ocs2_gazebo::JointCommandArray WbcController::getCommandMessage(const scalar_t dt) {
    mrt_.spinMRT();
    mrt_.updatePolicy();

    tNow_ += dt;

    auto observation = stateSubscriberPtr_->getObservation();

    ocs2::vector_t desiredState;
    ocs2::vector_t desiredInput;
    size_t desiredMode;
    mrt_.evaluatePolicy(tNow_, observation.state, desiredState, desiredInput, desiredMode);

    constexpr ocs2::scalar_t time_eps = 1e-4;
    ocs2::vector_t dummyState;
    ocs2::vector_t dummyInput;
    size_t dummyMode;
    mrt_.evaluatePolicy(tNow_ + time_eps, observation.state, dummyState, dummyInput, dummyMode);

    ocs2::vector_t joint_accelerations = (dummyInput.tail<12>() - desiredInput.tail<12>()) / time_eps;

    auto commandMessage = sqpWbc_.getCommandMessage(tNow_, observation.state, observation.input, observation.mode,
                                                    desiredState, desiredInput, desiredMode, joint_accelerations);

    timeSinceLastMpcUpdate_ += dt;
    if (timeSinceLastMpcUpdate_ >= 1.0 / mpcRate_) {
        setObservation();
    }

    return commandMessage;
}

void WbcController::setObservation() {
    mrt_.setCurrentObservation(stateSubscriberPtr_->getObservation());
    timeSinceLastMpcUpdate_ = 0.0;
}

bool WbcController::isSupported(const std::string &controllerType) {
    if (controllerType == "WBC") {
        return true;
    }
    return false;
}

}  // namespace ocs2_control