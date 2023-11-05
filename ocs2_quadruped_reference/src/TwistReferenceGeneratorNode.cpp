#include <iostream>
#include <string>
#include <memory>

#include <ros/ros.h>

#include <ocs2_core/Types.h>
#include "ocs2_quadruped_reference/ReferenceGenerator.h"

int main(int argc, char *argv[]) {
    const std::string robotName = "anymal";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_reference_generator_node");
    ros::NodeHandle nh;

    // Get parameters
    std::vector<std::string> programArgs{};
    ros::removeROSArgs(argc, argv, programArgs);
    const std::string targetCommandFile(programArgs[1]);

    // Initialize twist controller
    const std::string twistTopic = "/twist_cmd";
    const ocs2::scalar_t axisVel = 0.5;
    ROS_INFO_STREAM("Initializing twist controller with topic: " << twistTopic);
    auto twistCommandControllerPtr = std::unique_ptr<switched_model::CommandController>(
        new switched_model::TwistCommandController(targetCommandFile, nh, twistTopic, axisVel));

    // Initialize reference generator
    const std::string observationTopic = "/anymal_mpc_observation";
    const std::string referenceTopic = "/anymal_mpc_target";
    const ocs2::scalar_t dt = 0.1;
    const size_t N = 15;
    ROS_INFO_STREAM("Initializing reference generator");
    switched_model::ReferenceGenerator referenceGenerator(targetCommandFile, dt, N, observationTopic, referenceTopic,
                                                          nh);
    referenceGenerator.setCommandController(std::move(twistCommandControllerPtr));

    // Publish reference trajectory
    ros::Rate rate(10.0);
    while (ros::ok()) {
        ros::spinOnce();
        referenceGenerator.publishReferenceTrajectory();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}