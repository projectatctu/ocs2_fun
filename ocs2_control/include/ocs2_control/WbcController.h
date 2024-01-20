#pragma once

#include <ocs2_control/Controller.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_control/wbc/SqpWbc.h>
#include <ocs2_control/StateSubscriber.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>


#include <memory>
#include <ocs2_quadruped_interface/QuadrupedInterface.h>

namespace ocs2_control {
class WbcController : public Controller {
   public:
    WbcController(const std::string &configFile, const std::string &frameDeclarationFile,
                  const std::string &taskSettingsFile, const std::string &controllerConfigFile,
                  const std::string &urdfString, switched_model::SqpWbc &sqp,
                  std::shared_ptr<switched_model::QuadrupedVisualizer> visualizerPtr,
                  std::shared_ptr<StateSubscriber> stateSubscriberPtr);

    ~WbcController() override = default;

    ocs2_gazebo::JointCommandArray getCommandMessage(const scalar_t dt) override;

    void visualize() override;

    void changeController(const std::string &controllerType, const scalar_t time) override;

    bool isSupported(const std::string &controllerType) override;

   private:
    void resetMpc();

    void setObservation();

    ocs2_gazebo::JointCommandArray prepareMessage(const vector_t &jointAngles);

    ocs2_gazebo::JointCommandArray getStandMessage();
    ocs2_gazebo::JointCommandArray getSitMessage();
    ocs2_gazebo::JointCommandArray getInterpMessage(const scalar_t dt);

    scalar_t kp_;
    scalar_t kd_;

    // Stand joint angles
    ocs2::vector_t standJointAngles_;

    // Sit joint angles
    ocs2::vector_t sitJointAngles_;

    // Interpolation helper variables
    scalar_t alpha_;
    vector_t interpolateFrom_;
    vector_t interpolateTo_;

    std::string controllerType_;
    scalar_t interpolationTime_;

    std::vector<std::string> jointNames_;

    std::shared_ptr<StateSubscriber> stateSubscriberPtr_;

    ocs2::MRT_ROS_Interface mrt_;
    bool mrt_initialized_ = false;

    scalar_t mpcRate_ = 30.0;
    scalar_t timeSinceLastMpcUpdate_ = 1e5;

    switched_model::SqpWbc &sqpWbc_;

    scalar_t tNow_;

    std::shared_ptr<switched_model::QuadrupedVisualizer> visualizerPtr_;
};

}  // namespace ocs2_control