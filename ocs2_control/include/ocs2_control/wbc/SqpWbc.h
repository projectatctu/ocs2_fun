#pragma once

#include <ocs2_core/Types.h>

#include <ocs2_control/wbc/SqpSolver.h>
#include <ocs2_control/wbc/WbcBase.h>

#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/QuadrupedKinematics.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_gazebo/JointCommandArray.h>

namespace switched_model {

class SqpWbc : public WbcBase {
   public:
    SqpWbc(const std::string &configFile, const std::string &urdfString,
           const switched_model::ComModelBase<scalar_t> &comModel,
           const switched_model::KinematicsModelBase<scalar_t> &kinematics, const std::vector<std::string> &jointNames)
        : WbcBase(configFile, urdfString, comModel, kinematics), jointNames_(jointNames) {
        loadSettings(configFile);
    }

    ocs2_gazebo::JointCommandArray getCommandMessage(scalar_t currentTime, const vector_t &currentState,
                                                            const vector_t &currentInput, const size_t currentMode,
                                                            const vector_t &desiredState, const vector_t &desiredInput,
                                                            const size_t desiredMode,
                                                            const vector_t &desiredJointAcceleration);

   private:
    void loadSettings(const std::string &configFile);

    // Parameters
    scalar_t weightBaseAcceleration_;
    scalar_t weightContactForce_;
    scalar_t weightSwingLeg_;

    // joint kp and kd for swing legs
    scalar_t jointSwingKp_;
    scalar_t jointSwingKd_;

    // joint kp and kd for stance legs
    scalar_t jointStanceKp_;
    scalar_t jointStanceKd_;

    SqpSolver sqpSolver_;
    std::vector<std::string> jointNames_;
};

}  // namespace switched_model
