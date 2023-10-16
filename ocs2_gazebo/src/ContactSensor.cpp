#include "ocs2_gazebo/ContactSensor.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

namespace gazebo {
GZ_REGISTER_SENSOR_PLUGIN(ContactSensor)

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ContactSensor::Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) {
    this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);

    if (!this->parentSensor) {
        gzerr << "ContactPlugin requires a ContactSensor.\n";
        return;
    }

    this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&ContactSensor::OnUpdate, this));

    this->parentSensor->SetActive(true);

    ros::NodeHandle nh;
    std::string configFile;
    nh.getParam("/pluginConfigFile", configFile);
    loadSettings(configFile);

    // create ROS publisher
    this->contact_publisher = ros::NodeHandle().advertise<std_msgs::Bool>(this->parentSensor->Name(), 1);

    // Set initial state
    this->last_state = false;

    std::cout << "ContactSensor loaded" << std::endl;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ContactSensor::OnUpdate() {
    bool state = parentSensor->Contacts().contact_size() != 0;
    if (state == last_state) {
        return;
    }
    contact_msg_.data = state;
    contact_publisher.publish(contact_msg_);
    last_state = state;
}

/**********************************************************************************************************************/
/**********************************************************************************************************************/
/**********************************************************************************************************************/
void ContactSensor::loadSettings(const std::string &configFile) {
    using ocs2::scalar_t;
    using ocs2::loadData::loadCppDataType;

    const std::string prefix = "contactSensor.";

    // load rate
    scalar_t rate;
    loadCppDataType<scalar_t>(configFile, prefix + "rate", rate);
    std::cout << "ContactSensor rate: " << rate << std::endl;
    this->parentSensor->SetUpdateRate(rate);
}

};  // namespace gazebo