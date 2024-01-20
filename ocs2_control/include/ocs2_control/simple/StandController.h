#pragma once

#include <string>
#include <vector>

#include <ocs2_gazebo/JointCommandArray.h>

#include <ocs2_core/Types.h>

namespace switched_model {
class StandController {
   public:
    StandController(const std::string &controllerConfigFile, const std::string &configFile,
                    const std::vector<std::string> &jointNames);
    ocs2_gazebo::JointCommandArray getCommandMessage(const ocs2::vector_t &currentState);
    const ocs2::vector_t &getDefaultJointState() const { return defaultJointState_; }

   private:
    void loadSettings(const std::string &controllerConfigFile);

    ocs2::scalar_t jointKp_;
    ocs2::scalar_t jointKd_;

    std::vector<std::string> jointNames_;
    ocs2::vector_t defaultJointState_;
};
}  // namespace switched_model