#pragma once

#include <string>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_anymal_commands/ReferenceExtrapolation.h>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace switched_model {

using ocs2::scalar_t;
using ocs2::SystemObservation;

struct VelocityCommand {
    scalar_t velocity_x;
    scalar_t velocity_y;
    scalar_t yaw_rate;
};

class CommandController {
   public:
    virtual VelocityCommand getVelocityCommand(scalar_t time) = 0;
};

class JoystickController : public CommandController {
   public:
    JoystickController(const std::string &targetCommandFile, ros::NodeHandle &nh, const std::string &joyTopic, scalar_t axisVel);
    VelocityCommand getVelocityCommand(scalar_t time) override;

   private:
    void joystickCallback(const sensor_msgs::Joy::ConstPtr &msg);
    void loadSettings(const std::string &targetCommandFile);

    ros::Subscriber joystickSubscriber_;
    scalar_t axisVel_;

    scalar_t velocity_x_;
    scalar_t velocity_y_;
    scalar_t yaw_rate_;
    scalar_t linearVelocity_;
    scalar_t angularVelocity_;

};

class ReferenceGenerator {
   public:
    ReferenceGenerator(const std::string &targetCommandFile, scalar_t dt, size_t N, const std::string &observationTopic,
                       const std::string &referenceTopic, ros::NodeHandle &nh);
    void setCommandController(std::unique_ptr<CommandController> &&commandControllerPtr);
    void publishReferenceTrajectory();

   private:
    BaseReferenceHorizon getBaseReferenceHorizon();
    BaseReferenceCommand getBaseReferenceCommand(scalar_t time);
    BaseReferenceState getBaseReferenceState();
    TerrainPlane &getTerrainPlane();
    ocs2::TargetTrajectories generateReferenceTrajectory(scalar_t time);

    void loadSettings(const std::string &targetCommandFile);

    // ROS callbacks
    ros::Subscriber observationSubscriber_;
    void observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg);
    
    ros::Subscriber terrainSubscriber_;
    void terrainCallback(const grid_map_msgs::GridMap &msg);

    ros::Publisher referencePublisher_;

    ocs2::vector_t defaultJointState_;
    TerrainPlane localTerrain_;
    bool firstObservationReceived_;
    scalar_t comHeight_;
    scalar_t dt_;  // timestep
    size_t N_;     // number of timesteps in reference horizon
    std::mutex observationMutex_;
    std::mutex terrainMutex_;
    std::unique_ptr<CommandController> commandControllerPtr_;
    std::unique_ptr<grid_map::GridMap> terrainMapPtr_;
    SystemObservation latestObservation_;
};

}  // namespace switched_model