#include <string>
#include <vector>

#include "ocs2_gazebo/JointController.h"

namespace switched_model {

JointController::~JointController() {
    // unsubscriber from WBC control command topic
    command_subscriber.shutdown();
}

bool JointController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {
    ROS_INFO("[Switched model joint controller] Initializing joint controller.");

    // get JointConstSharedPtrs
    const std::string joints_tag = "joints";
    std::vector<std::string> joint_names;
    if (!n.getParam(joints_tag, joint_names)) {
        ROS_ERROR("Could not get joint names! Failed to initialize.");
        return false;
    }

    std::string descriptionName;
    n.getParam("/description_name", descriptionName);

    urdf::Model urdf;
    if (!urdf.initParam(descriptionName)) {
        ROS_ERROR("Could not parse urdf file! Failed to initialize.");
        return false;
    }

    for (int i = 0; i < joint_names.size(); ++i) {
        auto joint_name = joint_names[i];
        try {
            joint_map[joint_name] = hw->getHandle(joint_name);
        } catch (const hardware_interface::HardwareInterfaceException &e) {
            ROS_ERROR("Exception thrown %s", e.what());
            return false;
        }

        std::pair<double, double> joint_limit;
        urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_name);
        if (!joint_urdf) {
            ROS_ERROR("Could not find joint '%s' in urdf", joint_name.c_str());
            return false;
        }
        joint_limit.first = joint_urdf->limits->lower;
        joint_limit.second = joint_urdf->limits->upper;
        joint_limits[joint_name] = joint_limit;

        ROS_INFO_STREAM("Loaded joint controller for joint: " << joint_name);
    }

    // subscribe to WBC control command topic
    // TODO: change topic name - load this topic name from a config file
    const std::string topic = "switched_model/command";
    command_subscriber =
        n.subscribe<ocs2_gazebo::JointCommandArray>(topic, 1, &JointController::command_callback, this);

    // Initialize real-time buffer
    command_buffer.writeFromNonRT(BufferType());

    return true;
}

void JointController::update(const ros::Time &time, const ros::Duration &period) {
    BufferType &command_array = *command_buffer.readFromRT();

    const int n_commands = command_array.joint_commands.size();
    for (int i = 0; i < n_commands; ++i) {
        ocs2_gazebo::JointCommand &command = command_array.joint_commands[i];

        // unpack message
        std::string joint_name = command.joint_name;
        double position_desired = command.position_desired;
        double velocity_desired = command.velocity_desired;
        double kp = command.kp;
        double kd = command.kd;
        double torque_ff = command.torque_ff;

        // get current values
        hardware_interface::JointHandle &joint = joint_map[joint_name];
        double position_current = joint.getPosition();
        double velocity_current = joint.getVelocity();

        // compute position and velocity error
        double position_error = position_desired - position_current;
        double velocity_error = velocity_desired - velocity_current;

        // compute torque command
        double torque = torque_ff + kp * position_error + kd * velocity_error;

        // set joint torque
        joint.setCommand(torque);

        // Todo: enforce joint limits
    }
}

void JointController::command_callback(const ocs2_gazebo::JointCommandArrayConstPtr &command_ptr) {
    command_buffer.writeFromNonRT(*command_ptr);
}

}  // namespace switched_model

PLUGINLIB_EXPORT_CLASS(switched_model::JointController, controller_interface::ControllerBase);
