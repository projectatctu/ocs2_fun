#pragma once

#include <ocs2_control/wbc/Task.h>

#include <ocs2_anymal_models/QuadrupedCom.h>
#include <ocs2_anymal_models/QuadrupedKinematics.h>

namespace switched_model {

class WbcBase {
   public:
    WbcBase(const std::string &configFile, const std::string &urdfString,
            const switched_model::ComModelBase<scalar_t> &comModel,
            const switched_model::KinematicsModelBase<scalar_t> &kinematics);

   protected:
    Task createDynamicsTask();
    Task createContactForceTask();
    Task createStanceFootNoMotionTask();
    Task createTorqueLimitTask();

    Task createBaseAccelerationTask(const vector_t &stateCurrent, const vector_t &stateDesired,
                                    const vector_t &inputDesired, const vector_t &desiredJointAcceleration);
    Task createSwingFootAccelerationTask(const vector_t &stateCurrent, const vector_t &inputCurrent,
                                         const vector_t &stateDesired, const vector_t &inputDesired);
    Task createContactForceMinimizationTask(const vector_t &inputDesired);

    void updateMeasuredState(const vector_t &stateCurrent, const vector_t &inputCurrent);
    void updateKinematicsAndDynamicsCurrent();
    void updateDesiredState(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateKinematicsAndDynamicsDesired(const vector_t &stateDesired, const vector_t &inputDesired);
    void updateContactFlags(const size_t modeCurrent, const size_t modeDesired);

    /* Contact jacobians - stacked on top of each other */
    matrix_t Jcontact_;

    /* Contact jacobians time derivative - stacked on top of each other */
    matrix_t dJcontactdt_;

    /* Measured generalized positions */
    vector_t qMeasured_;

    /* Measured generalized velocities */
    vector_t vMeasured_;

    /* Measured rotation matrix from base to world*/
    matrix_t rMeasured_;

    /* Desired rotation matrix from base to world*/
    matrix_t rDesired_;

    /* Number of decision variables */
    size_t nDecisionVariables_;

    /* Number of generalized coordinates */
    size_t nGeneralizedCoordinates_;

    /* Measured pinocchio interface */
    ocs2::PinocchioInterface pinocchioInterfaceMeasured_;

    /* Foot names*/
    std::vector<std::string> footNames_;

    /* Friction cone matrix*/
    matrix_t muMatrix_;

    /* swing kp and kd*/
    scalar_t swingKp_;
    scalar_t swingKd_;

    /* base kp and kd */
    scalar_t baseKp_;
    scalar_t baseKd_;

    /* euler kp and kd*/
    scalar_t eulerKp_;
    scalar_t eulerKd_;

    /* torque limit */
    scalar_t torqueLimit_;

    /* Desired contact flags */
    contact_flag_t contactFlags_;
    size_t nContacts_;

    /* Measured contact flags*/
    contact_flag_t contactFlagsCurrent_;

   private:
    void loadSettings(const std::string &configFile);
    void generateFrictionConeMatrix(const scalar_t mu);

    std::unique_ptr<switched_model::ComModelBase<scalar_t>> comModelPtr_;
    std::unique_ptr<switched_model::KinematicsModelBase<scalar_t>> kinematicsPtr_;
};

}  // namespace switched_model