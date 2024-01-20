#pragma once

#include <ocs2_control/Controller.h>
#include <ocs2_control/StateSubscriber.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>


#include <memory>

namespace ocs2_control {
class StaticController : public Controller {
   public:
    StaticController(const std::string &configFile, std::shared_ptr<switched_model::QuadrupedVisualizer> visualizerPtr,
                     std::shared_ptr<StateSubscriber> stateSubscriberPtr);

    ~StaticController() override = default;

    ocs2_gazebo::JointCommandArray getCommandMessage(const scalar_t dt) override;

    void visualize() override;

    void changeController(const std::string &controllerType, const scalar_t time) override;

    bool isSupported(const std::string &controllerType) override;

   private:
    void loadSettings(const std::string &configFile);

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
    std::shared_ptr<switched_model::QuadrupedVisualizer> visualizerPtr_;

    scalar_t timeSinceLastUpdate_ = 0.0;
};
}  // namespace ocs2_control
