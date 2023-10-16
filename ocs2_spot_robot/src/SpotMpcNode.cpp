/*
 * AnymalMPC.cpp
 *
 *  Created on: Apr 15, 2018
 *      Author: farbod
 */

#include <ocs2_quadruped_interface/QuadrupedMpcNode.h>
#include <ros/init.h>

#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_quadruped_interface/QuadrupedMpc.h>

#include <ocs2_anymal_mpc/AnymalInterface.h>

int main(int argc, char *argv[]) {
    // Initialize ros node
    ros::init(argc, argv, "anymal_mpc");
    ros::NodeHandle nodeHandle;

    // Description name
    std::string descriptionName;
    nodeHandle.getParam("/description_name", descriptionName);

    // Spot urdf
    std::string urdfString;
    nodeHandle.getParam(descriptionName, urdfString);

    // Task settings
    std::string taskSettingsFile;
    nodeHandle.getParam("/task_settings_file", taskSettingsFile);

    // Frame declarations
    std::string frameDeclarationFile;
    nodeHandle.getParam("/frame_declaration_file", frameDeclarationFile);

    // SQP settings
    std::string sqpSettingsFile;
    nodeHandle.getParam("/sqp_settings_file", sqpSettingsFile);

    auto spotInterface =
        anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                   anymal::frameDeclarationFromFile(frameDeclarationFile));
    const auto mpcSettings = ocs2::mpc::loadSettings(taskSettingsFile);

    if (spotInterface->modelSettings().algorithm_ != switched_model::Algorithm::SQP) {
        throw std::runtime_error("Only SQP is supported. Aborting.");
    }

    const auto sqpSettings = ocs2::sqp::loadSettings(sqpSettingsFile);
    auto mpcPtr = switched_model::getSqpMpc(*spotInterface, mpcSettings, sqpSettings);
    switched_model::quadrupedMpcNode(nodeHandle, *spotInterface, std::move(mpcPtr));

    return EXIT_SUCCESS;
}
