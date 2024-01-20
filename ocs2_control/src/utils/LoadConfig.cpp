#include "ocs2_control/utils/LoadConfig.h"

#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

ocs2::vector_t getDefaultJointPosition(const std::string &configFile, const std::string &prefix) {
    ocs2::vector_t defaultJointState;
    defaultJointState.setZero(12);
    ocs2::loadData::loadEigenMatrix(configFile, prefix + "defaultJointState", defaultJointState);
    return defaultJointState;
}

}  // namespace switched_model