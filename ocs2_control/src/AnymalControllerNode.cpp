#include <pinocchio/fwd.hpp>

#include <string>
#include <array>
#include <chrono>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <ocs2_control/simple/StandController.h>
#include <ocs2_control/wbc/SqpWbc.h>

#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

#include <ocs2_mpc/MPC_Settings.h>
#include "ocs2_anymal_mpc/AnymalInterface.h"
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

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

    // Command publisher
    const std::string commandTopic = "/anymal_joint_controller/joint_controller/switched_model/command";
    ros::Publisher commandPublisher = nh.advertise<ocs2_gazebo::JointCommandArray>(commandTopic, 1, true);

    // Description name
    std::string descriptionName;
    nh.getParam("/description_name", descriptionName);

    // Anymal urdf
    std::string urdfString;
    nh.getParam(descriptionName, urdfString);

    // Task settings
    std::string taskSettingsFile;
    nh.getParam("/task_settings_file", taskSettingsFile);

    // Frame declarations
    std::string frameDeclarationFile;
    nh.getParam("/frame_declaration_file", frameDeclarationFile);

    auto anymalInterface =
        anymal::getAnymalInterface(urdfString, switched_model::loadQuadrupedSettings(taskSettingsFile),
                                   anymal::frameDeclarationFromFile(frameDeclarationFile));
    ocs2::MRT_ROS_Interface mrt(robotName);
    mrt.launchNodes(nh);
    std::string controllerConfigFile;
    nh.getParam("/controller_config", controllerConfigFile);

    // Stand controller
    switched_model::StandController standController(controllerConfigFile, targetCommandConfig,
                                                    anymalInterface->getJointNames());

    // Whole body controller
    switched_model::SqpWbc sqpWbc(controllerConfigFile, urdfString, anymalInterface->getComModel(),
                                  anymalInterface->getKinematicModel(), anymalInterface->getJointNames());

    // Initialize visualizer
    auto visualizer = std::make_shared<switched_model::QuadrupedVisualizer>(
        anymalInterface->getKinematicModel(), anymalInterface->getJointNames(), anymalInterface->getBaseName(), nh);

    // initial state
    ocs2::SystemObservation initObservation;
    initObservation.state = anymalInterface->getInitialState();
    initObservation.input = ocs2::vector_t::Zero(switched_model::INPUT_DIM);
    initObservation.mode = switched_model::ModeNumber::STANCE;

    auto commands = standController.getCommandMessage();
    commandPublisher.publish(commands);
    ros::Duration(2.5).sleep();

    // initial command
    const ocs2::TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state}, {initObservation.input});
    mrt.resetMpcNode(initTargetTrajectories);

    // wait for initial state message
    const std::string stateTopic = "/" + descriptionName + "/" + "anymal" + "/state";
    ocs2_gazebo::RobotState robotState;
    auto stateCallback = [&robotState](const ocs2_gazebo::RobotState::ConstPtr &msg) { robotState = *msg; };
    ros::Subscriber stateSubscriber = nh.subscribe<ocs2_gazebo::RobotState>(stateTopic, 1, stateCallback);
    ROS_INFO("Waiting for initial state message...");
    auto stateMsg = ros::topic::waitForMessage<ocs2_gazebo::RobotState>(stateTopic, nh);
    stateCallback(stateMsg);

    // Wait for initial policy
    while (!mrt.initialPolicyReceived() && ros::ok() && ros::master::check()) {
        ROS_INFO("Waiting for initial policy...");
        mrt.spinMRT();
        mrt.setCurrentObservation(initObservation);
        ros::Duration(0.1).sleep();
    }

    ROS_INFO("Initial policy received.");

    auto initial_time = ros::Time::now();

    // ocs2 Observation publisher
    ros::Rate rate(400);
    int i = 0;
    int j = 0;

    bool policyReceived = false;
    while (ros::ok()) {
        ros::spinOnce();
        mrt.spinMRT();

        if (!policyReceived && mrt.updatePolicy()) {
            policyReceived = true;
        }

        auto t_now = ros::Time::now().toSec() - initial_time.toSec();

        // Generate observation from robot state
        ocs2::SystemObservation observation;

        // Observation time
        observation.time = robotState.stamp.toSec() - initial_time.toSec();

        // Mode
        std::array<bool, 4> contacts;
        std::copy(robotState.contacts.begin(), robotState.contacts.end(), contacts.begin());
        observation.mode = switched_model::stanceLeg2ModeNumber(contacts);

        // State
        observation.state = Eigen::Map<ocs2::vector_t>(robotState.x.data(), robotState.x.size());

        // Input
        observation.input.setZero(24);
        observation.input.tail<12>() = Eigen::Map<ocs2::vector_t>(robotState.u.data(), robotState.u.size());

        ocs2::vector_t desiredState;
        ocs2::vector_t desiredInput;
        size_t desiredMode;
        mrt.evaluatePolicy(t_now, observation.state, desiredState, desiredInput, desiredMode);

        constexpr ocs2::scalar_t time_eps = 1e-4;
        ocs2::vector_t dummyState;
        ocs2::vector_t dummyInput;
        size_t dummyMode;
        mrt.evaluatePolicy(t_now + time_eps, observation.state, dummyState, dummyInput, dummyMode);

        ocs2::vector_t joint_accelerations = (dummyInput.tail<12>() - desiredInput.tail<12>()) / time_eps;

        // std::cout << joint_accelerations.transpose() << std::endl << std::endl;

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        auto commandMessage2 = sqpWbc.getCommandMessage(t_now, observation.state, observation.input, observation.mode,
                                                        desiredState, desiredInput, desiredMode, joint_accelerations);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        if (j < 100) {
            auto commands = standController.getCommandMessage();
            commandPublisher.publish(commands);
            ++j;
        } else {
            commandPublisher.publish(commandMessage2);
        }

        if (i % 6 == 0) {
            mrt.setCurrentObservation(observation);
            policyReceived = false;
            visualizer->update(observation, mrt.getPolicy(), mrt.getCommand());
            i = 0;
        } else {
            ++i;
        }
        rate.sleep();
    }
    return EXIT_SUCCESS;
}