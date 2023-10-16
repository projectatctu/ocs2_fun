#include "ocs2_control/simple/StandController.h"

#include <ocs2_gazebo/JointCommand.h>
#include <ocs2_control/utils/LoadConfig.h>

#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
StandController::StandController(const std::string &controllerConfigFile, const std::string &configFile,
                                 const std::vector<std::string> &jointNames)
    : jointNames_(jointNames) {
    defaultJointState_ = getDefaultJointPosition(configFile);
    loadSettings(controllerConfigFile);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2_gazebo::JointCommandArray StandController::getCommandMessage() {
    ocs2_gazebo::JointCommandArray commandMessages;
    commandMessages.joint_commands.resize(jointNames_.size());
    for (size_t i = 0; i < jointNames_.size(); ++i) {
        ocs2_gazebo::JointCommand commandMessage;
        commandMessage.joint_name = jointNames_[i];
        commandMessage.position_desired = defaultJointState_(i);
        commandMessage.velocity_desired = 0.0;
        commandMessage.kp = jointKp_;
        commandMessage.kd = jointKd_;
        commandMessage.torque_ff = 0.0;
        commandMessages.joint_commands[i] = std::move(commandMessage);
    }
    return commandMessages;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StandController::loadSettings(const std::string &controllerConfigFile) {
    using ocs2::scalar_t;
    using ocs2::loadData::loadCppDataType;

    const std::string prefix = "standController.";

    // load gains
    loadCppDataType<scalar_t>(controllerConfigFile, prefix + "jointKp", jointKp_);
    loadCppDataType<scalar_t>(controllerConfigFile, prefix + "jointKd", jointKd_);
}


}  // namespace switched_model