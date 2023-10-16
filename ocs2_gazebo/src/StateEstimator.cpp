#include "ocs2_gazebo/StateEstimator.h"

#include <Eigen/Geometry>
#include <ocs2_anymal_commands/TerrainAdaptation.h>

#include <ocs2_core/misc/LoadData.h>

#include <string>

#define CONTACT_BUFFER_SIZE 1

namespace gazebo {

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StateEstimator::Load(physics::ModelPtr robot, sdf::ElementPtr sdf) {
    // set Gazebo callback function
    updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&StateEstimator::OnUpdate, this));

    // save robot's simulation model
    this->robot = robot;

    // set state publisher up
    ros::NodeHandle nh(robot->GetName());
    state_publisher = nh.advertise<ocs2_gazebo::RobotState>("anymal/state", 2);

    std::string configFile;
    nh.getParam("/pluginConfigFile", configFile);

    loadSettings(configFile);

    // get base link
    std::string base = "base";
    base_link = robot->GetChildLink(base);

    // get joints; ignore 'universe' and 'root_joint'
    std::vector<std::string> joint_names = {"LF_HAA", "LF_HFE", "LF_KFE", "RF_HAA", "RF_HFE", "RF_KFE",
                                            "LH_HAA", "LH_HFE", "LH_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
    for (int i = 0; i < joint_names.size(); ++i) {
        joints.push_back(robot->GetJoint(joint_names[i]));
    }

    // initialize last publish time
    last_publish_time = robot->GetWorld()->SimTime();

    // initialize contacts
    contacts.reserve(4);
    for (int i = 0; i < 4; ++i) {
        contacts[i] = false;
    }

    // subscribe to contact topics
    lf_sub = nh.subscribe("/lf_foot_contact", CONTACT_BUFFER_SIZE, &StateEstimator::lf_foot_contact_callback, this);
    rf_sub = nh.subscribe("/rf_foot_contact", CONTACT_BUFFER_SIZE, &StateEstimator::rf_foot_contact_callback, this);
    lh_sub = nh.subscribe("/lh_foot_contact", CONTACT_BUFFER_SIZE, &StateEstimator::lh_foot_contact_callback, this);
    rh_sub = nh.subscribe("/rh_foot_contact", CONTACT_BUFFER_SIZE, &StateEstimator::rh_foot_contact_callback, this);

    yaw_last = 0.0;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StateEstimator::OnUpdate() {
    common::Time current_time = robot->GetWorld()->SimTime();

    // update rate check
    if ((current_time - last_publish_time).Double() < (1.0 / rate)) {
        return;
    }

    // New representation
    ocs2_gazebo::RobotState state;

    /* Set contact states */
    state.contacts.resize(4);
    for (size_t i = 0; i < 4; ++i) {
        state.contacts[i] = contacts[i];
    }

    /* Set x*/
    state.x.reserve(3 + 3 + 3 + 3 + 12);
    state.u.reserve(12);

    ignition::math::Pose3d pose = base_link->WorldPose();

    // base orientation - quaternion
    Eigen::Quaternion<double> quat(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());

    // base orientation - Euler angles (ZYX)
    const Eigen::Matrix3d R_world_base = quat.toRotationMatrix();
    const Eigen::Matrix3d R_base_world = R_world_base.transpose();
    Eigen::Vector3d rpy = switched_model::eulerXYZFromRotationMatrix(R_world_base, yaw_last);
    yaw_last = rpy[2];

    // 0) Observation time
    state.stamp = ros::Time::now();

    // 1) Base orientation
    state.x.push_back(rpy[0]);
    state.x.push_back(rpy[1]);
    state.x.push_back(rpy[2]);

    // 2) Base position
    state.x.push_back(pose.Pos().X());
    state.x.push_back(pose.Pos().Y());
    state.x.push_back(pose.Pos().Z());

    // 3) Base angular velocity
    ignition::math::Vector3d angular_velocity = base_link->WorldAngularVel();
    Eigen::Vector3d angular_velocity_world(angular_velocity.X(), angular_velocity.Y(), angular_velocity.Z());
    Eigen::Vector3d angular_velocity_body = R_base_world * angular_velocity_world;
    state.x.push_back(angular_velocity_body[0]);
    state.x.push_back(angular_velocity_body[1]);
    state.x.push_back(angular_velocity_body[2]);

    // 4) Base linear velocity
    ignition::math::Vector3d linear_velocity = base_link->WorldLinearVel();
    Eigen::Vector3d linear_velocity_world(linear_velocity.X(), linear_velocity.Y(), linear_velocity.Z());
    Eigen::Vector3d linear_velocity_body = R_base_world * linear_velocity_world;
    state.x.push_back(linear_velocity_body[0]);
    state.x.push_back(linear_velocity_body[1]);
    state.x.push_back(linear_velocity_body[2]);

    // 5) Joint angles
    for (int i = 0; i < joints.size(); ++i) {
        state.x.push_back(joints[i]->Position(0));
    }

    // 6) Joint velocities
    for (int i = 0; i < joints.size(); ++i) {
        state.u.push_back(joints[i]->GetVelocity(0));
    }

    // publish robot state
    state_publisher.publish(state);

    // update publish time
    last_publish_time = current_time;
}

/*********************************************************************************************************************/
/*********************************************************************************************************************/
/*********************************************************************************************************************/
void StateEstimator::loadSettings(const std::string &configFile) {
    using ocs2::scalar_t;
    using ocs2::loadData::loadCppDataType;

    const std::string prefix = "stateEstimator.";

    // load rate
    loadCppDataType<scalar_t>(configFile, prefix + "rate", rate);
}

GZ_REGISTER_MODEL_PLUGIN(StateEstimator);
}  // namespace gazebo