#include <ocs2_control/StaticController.h>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2_control {

StaticController::StaticController(const std::string &configFile,
                                   std::shared_ptr<switched_model::QuadrupedVisualizer> visualizerPtr,
                                   std::shared_ptr<StateSubscriber> stateSubscriberPtr)
    : visualizerPtr_(visualizerPtr) {
    loadSettings(configFile);
    alpha_ = -1.0;
    stateSubscriberPtr_ = stateSubscriberPtr;
}

ocs2_gazebo::JointCommandArray StaticController::getCommandMessage(const scalar_t dt) {
    timeSinceLastUpdate_ += dt;
    if (alpha_ != -1.0) {
        return getInterpMessage(dt);
    }

    if (controllerType_ == "STAND") {
        return getStandMessage();
    }

    if (controllerType_ == "SIT") {
        return getSitMessage();
    }
}

void StaticController::visualize() {
    if (timeSinceLastUpdate_ >= 1.0 / 30.0) {
        visualizerPtr_->publishObservation(ros::Time::now(), stateSubscriberPtr_->getObservation());
        timeSinceLastUpdate_ = 0.0;
    }
}

void StaticController::changeController(const std::string &controllerType, const scalar_t time) {
    controllerType_ = controllerType;
    alpha_ = 0.0;
    const auto &currentObservation = stateSubscriberPtr_->getObservation();

    interpolateFrom_ = currentObservation.state.tail<12>();
    if (controllerType_ == "STAND") {
        interpolateTo_ = standJointAngles_;
    } else if (controllerType_ == "SIT") {
        interpolateTo_ = sitJointAngles_;
    } else {
        throw std::runtime_error("Unsupported controller type: " + controllerType_);
    }
}

bool StaticController::isSupported(const std::string &controllerType) {
    if (controllerType == "STAND" || controllerType == "SIT") {
        return true;
    }
    return false;
}

void StaticController::loadSettings(const std::string &configFile) {
    const std::string prefix = "StaticController.";

    // Load PD gains
    ocs2::loadData::loadCppDataType<scalar_t>(configFile, prefix + "kp", kp_);
    ocs2::loadData::loadCppDataType<scalar_t>(configFile, prefix + "kd", kd_);

    // Load joint names, TODO: Move these names to a config file
    jointNames_ = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                   "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};

    standJointAngles_.setZero(12, 1);
    // Load stand joint angles
    ocs2::loadData::loadEigenMatrix(configFile, prefix + "standJointAngles", standJointAngles_);

    sitJointAngles_.setZero(12, 1);
    // Load sit joint angles
    ocs2::loadData::loadEigenMatrix(configFile, prefix + "sitJointAngles", sitJointAngles_);

    // Load interpolation time
    ocs2::loadData::loadCppDataType<scalar_t>(configFile, prefix + "interpolationTime", interpolationTime_);
}

ocs2_gazebo::JointCommandArray StaticController::prepareMessage(const vector_t &jointAngles) {
    ocs2_gazebo::JointCommandArray commandMessages;
    commandMessages.joint_commands.resize(jointNames_.size());
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        ocs2_gazebo::JointCommand commandMessage;
        commandMessage.joint_name = jointNames_[i];
        commandMessage.position_desired = jointAngles(i);
        commandMessage.velocity_desired = 0.0;
        commandMessage.kp = kp_;
        commandMessage.kd = kd_;
        commandMessage.torque_ff = 0.0;
        commandMessages.joint_commands[i] = std::move(commandMessage);
    }
    return commandMessages;
}

ocs2_gazebo::JointCommandArray StaticController::getStandMessage() { return prepareMessage(standJointAngles_); }

ocs2_gazebo::JointCommandArray StaticController::getSitMessage() { return prepareMessage(sitJointAngles_); }

ocs2_gazebo::JointCommandArray StaticController::getInterpMessage(const scalar_t dt) {
    alpha_ = std::min(alpha_ + dt / interpolationTime_, 1.0);
    auto message = prepareMessage(alpha_ * interpolateTo_ + (1.0 - alpha_) * interpolateFrom_);

    if (alpha_ == 1.0) {
        alpha_ = -1.0;
    }
    return message;
}

}  // namespace ocs2_control