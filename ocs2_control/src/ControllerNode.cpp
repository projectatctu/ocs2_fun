#include <pinocchio/fwd.hpp>

#include <string>
#include <array>
#include <chrono>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <ocs2_control/simple/StandController.h>
#include <ocs2_control/wbc/SqpWbc.h>

#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

#include <ocs2_core/misc/LoadData.h>

#include <ocs2_mpc/MPC_Settings.h>
#include "ocs2_anymal_mpc/AnymalInterface.h"
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_control/StateSubscriber.h>
#include <ocs2_control/CentralController.h>
#include <ocs2_control/StaticController.h>
#include <ocs2_control/WbcController.h>

#include <ocs2_gazebo/RobotState.h>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

int main(int argc, char *argv[]) {
    const std::string robotName = "anymal";

    // Initialize ROS
    ros::init(argc, argv, robotName + "_controller_node");
    ros::NodeHandle nh;

    // Load default joint state
    std::string targetCommandConfig;
    nh.getParam("/target_command_config", targetCommandConfig);

    // Description name
    std::string descriptionName;
    nh.getParam("/description_name", descriptionName);

    // URDF
    std::string urdfString;
    nh.getParam(descriptionName, urdfString);

    // Task settings
    std::string taskSettingsFile;
    nh.getParam("/task_settings_file", taskSettingsFile);

    // Frame declarations
    std::string frameDeclarationFile;
    nh.getParam("/frame_declaration_file", frameDeclarationFile);

    // Controller config
    std::string controllerConfigFile;
    nh.getParam("/controller_config", controllerConfigFile);

    // State subsciber
    const std::string stateTopic = "/" + descriptionName + "/" + robotName + "/state";
    auto stateSubscriber = std::make_shared<ocs2_control::StateSubscriber>(nh, stateTopic, 0.0);

    // Quadruped interface
    std::unique_ptr<switched_model::QuadrupedInterface> quadrupedInterfacePtr_ =
        anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                   anymal::frameDeclarationFromFile(frameDeclarationFile));

    // Visualizer
    auto visualizer = std::make_shared<switched_model::QuadrupedVisualizer>(quadrupedInterfacePtr_->getKinematicModel(),
                                                                            quadrupedInterfacePtr_->getJointNames(),
                                                                            quadrupedInterfacePtr_->getBaseName(), nh);

    // Static controller
    auto staticController =
        std::make_unique<ocs2_control::StaticController>(controllerConfigFile, visualizer, stateSubscriber);

    // WBC controller
    switched_model::SqpWbc sqpWbc(controllerConfigFile, urdfString, quadrupedInterfacePtr_->getComModel(),
                                  quadrupedInterfacePtr_->getKinematicModel(), quadrupedInterfacePtr_->getJointNames());
    auto wbcController = std::make_unique<ocs2_control::WbcController>(controllerConfigFile, frameDeclarationFile,
                                                                       taskSettingsFile, controllerConfigFile,
                                                                       urdfString, sqpWbc, visualizer, stateSubscriber);

    // Central controller
    ocs2_control::CentralController centralController;
    centralController.addController(std::move(staticController));
    centralController.addController(std::move(wbcController));

    // Wait for initial state message
    stateSubscriber->waitTillInitialized();

    ocs2::scalar_t freq;
    ocs2::loadData::loadCppDataType<ocs2::scalar_t>(taskSettingsFile, "mpc.mrtDesiredFrequency", freq);
    ros::Rate rate(freq);

    ros::Time lastTime = ros::Time::now();
    while (ros::ok()) {
        // Trigger callbacks
        ros::spinOnce();

        // Compute time since last step
        ros::Time currentTime = ros::Time::now();
        const double dt = (currentTime - lastTime).toSec();

        // Step controller
        centralController.step(dt);

        // Visualize
        centralController.visualize();

        // Update time and sleep
        lastTime = currentTime;
        rate.sleep();
    }

    ROS_INFO("Exiting... Bye!");

    return EXIT_SUCCESS;
}