#include "ocs2_quadruped_reference/ReferenceGenerator.h"

#include <ocs2_msgs/mpc_target_trajectories.h>
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include <ocs2_switched_model_interface/core/Rotations.h>

#include <grid_map_ros/grid_map_ros.hpp>

namespace switched_model {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void JoystickCommandController::messageCallback(const sensor_msgs::Joy &msg) {
    // Unpack message
    scalar_t x_multiplier = msg.axes[1];
    scalar_t y_multiplier = msg.axes[0];
    scalar_t yaw_multiplier = msg.axes[3];

    // Normalize linear velocity vector
    const scalar_t xy_size = std::sqrt(std::pow(x_multiplier, 2) + std::pow(y_multiplier, 2));
    if (xy_size > 1.0) {
        x_multiplier /= xy_size;
        y_multiplier /= xy_size;
    }

    // Set velocity command
    velocity_x_ = x_multiplier * linearVelocity_;
    velocity_y_ = y_multiplier * linearVelocity_;
    yaw_rate_ = yaw_multiplier * angularVelocity_;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void TwistCommandController::messageCallback(const geometry_msgs::Twist &msg) {
    velocity_x_ = msg.linear.x;
    velocity_y_ = msg.linear.y;
    yaw_rate_ = msg.angular.z;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ReferenceGenerator::ReferenceGenerator(const std::string &targetCommandFile, scalar_t dt, size_t N,
                                       const std::string &observationTopic, const std::string &referenceTopic,
                                       ros::NodeHandle &nh)
    : dt_(dt), N_(N), firstObservationReceived_(false) {
    defaultJointState_.setZero(12);
    loadSettings(targetCommandFile);

    // Setup ROS subscribers
    observationSubscriber_ = nh.subscribe(observationTopic, 1, &ReferenceGenerator::observationCallback, this);

    const std::string terrainTopic = "/elevation_mapping/elevation_map_raw";
    terrainSubscriber_ = nh.subscribe(terrainTopic, 1, &ReferenceGenerator::terrainCallback, this);

    // Setup ROS publishers
    referencePublisher_ = nh.advertise<ocs2_msgs::mpc_target_trajectories>(referenceTopic, 1, false);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceGenerator::setCommandController(std::unique_ptr<CommandController> &&commandControllerPtr) {
    commandControllerPtr_ = std::move(commandControllerPtr);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/

void ReferenceGenerator::publishReferenceTrajectory() {
    if (!firstObservationReceived_) {
        ROS_WARN_THROTTLE(1.0, "No observation received yet. Cannot publish reference trajectory.");
        return;
    }

    // Generate reference trajectory
    ocs2::TargetTrajectories referenceTrajectory = generateReferenceTrajectory(ros::Time::now().toSec());

    // Publish reference trajectory
    referencePublisher_.publish(ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(referenceTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceHorizon ReferenceGenerator::getBaseReferenceHorizon() { return {dt_, N_}; }

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceCommand ReferenceGenerator::getBaseReferenceCommand(scalar_t time) {
    const auto velocityCommand = commandControllerPtr_->getVelocityCommand(time);
    return {velocityCommand.velocity_x, velocityCommand.velocity_y, velocityCommand.yaw_rate, comHeight_};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
BaseReferenceState ReferenceGenerator::getBaseReferenceState() {
    std::lock_guard<std::mutex> lock(observationMutex_);
    scalar_t observationTime = latestObservation_.time;
    Eigen::Vector3d positionInWorld = latestObservation_.state.segment<3>(3);
    Eigen::Vector3d eulerXyz = latestObservation_.state.head<3>();
    return {observationTime, positionInWorld, eulerXyz};
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
TerrainPlane &ReferenceGenerator::getTerrainPlane() { return localTerrain_; }

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
ocs2::TargetTrajectories ReferenceGenerator::generateReferenceTrajectory(scalar_t time) {
    // Get base reference trajectory
    BaseReferenceTrajectory baseReferenceTrajectory;
    if (!terrainMapPtr_) {
        baseReferenceTrajectory = generateExtrapolatedBaseReference(getBaseReferenceHorizon(), getBaseReferenceState(),
                                                                    getBaseReferenceCommand(time), getTerrainPlane());

    } else {
        baseReferenceTrajectory =
            generateExtrapolatedBaseReference(getBaseReferenceHorizon(), getBaseReferenceState(),
                                              getBaseReferenceCommand(time), *terrainMapPtr_, 0.5, 0.3);
    }

    // Generate target trajectory
    ocs2::scalar_array_t desiredTimeTrajectory = std::move(baseReferenceTrajectory.time);
    const size_t N = desiredTimeTrajectory.size();
    ocs2::vector_array_t desiredStateTrajectory(N);
    ocs2::vector_array_t desiredInputTrajectory(N, ocs2::vector_t::Zero(INPUT_DIM));
    for (size_t i = 0; i < N; ++i) {
        ocs2::vector_t state = ocs2::vector_t::Zero(STATE_DIM);

        // base orientation
        state.head<3>() = baseReferenceTrajectory.eulerXyz[i];

        auto Rt = switched_model::rotationMatrixOriginToBase(baseReferenceTrajectory.eulerXyz[i]);

        // base position
        state.segment<3>(3) = baseReferenceTrajectory.positionInWorld[i];

        // base angular velocity
        state.segment<3>(6) = Rt * baseReferenceTrajectory.angularVelocityInWorld[i];

        // base linear velocity
        state.segment<3>(9) = Rt * baseReferenceTrajectory.linearVelocityInWorld[i];

        // joint angles
        state.segment<12>(12) = defaultJointState_;

        desiredStateTrajectory[i] = std::move(state);
    }

    return ocs2::TargetTrajectories(std::move(desiredTimeTrajectory), std::move(desiredStateTrajectory),
                                    std::move(desiredInputTrajectory));
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceGenerator::loadSettings(const std::string &targetCommandFile) {
    // Load target COM height
    ocs2::loadData::loadCppDataType<scalar_t>(targetCommandFile, "comHeight", comHeight_);

    // Load default joint angles
    ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", defaultJointState_);
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceGenerator::observationCallback(const ocs2_msgs::mpc_observation::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(observationMutex_);
    latestObservation_ = ocs2::ros_msg_conversions::readObservationMsg(*msg);
    firstObservationReceived_ = true;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void ReferenceGenerator::terrainCallback(const grid_map_msgs::GridMap &msg) {
    // Convert ROS message to grid map
    std::unique_ptr<grid_map::GridMap> mapPtr(new grid_map::GridMap);
    std::vector<std::string> layers = {"smooth_planar"};
    grid_map::GridMapRosConverter::fromMessage(msg, *mapPtr, layers, false, false);

    // Swap terrain map pointers
    terrainMapPtr_.swap(mapPtr);
}

}  // namespace switched_model