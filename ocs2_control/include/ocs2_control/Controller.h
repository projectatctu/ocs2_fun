#pragma once

#include <string>
#include <ocs2_core/Types.h>

#include <ocs2_gazebo/JointCommandArray.h>
#include <ocs2_gazebo/JointCommand.h>

namespace ocs2_control {

using ocs2::scalar_t;
using ocs2::vector_t;

class Controller {

    public:
        virtual ~Controller() = default;

        virtual ocs2_gazebo::JointCommandArray getCommandMessage(const scalar_t dt) = 0;

        virtual void visualize() = 0;

        virtual void changeController(const std::string &controllerType, const scalar_t time) = 0;

        virtual bool isSupported(const std::string &controllerType) = 0;
};

} // namespace ocs2_control