#ifndef INCLUDE_NMPC_CONTROLLER_NONLINEAR_MPC_H_
#define INCLUDE_NMPC_CONTROLLER_NONLINEAR_MPC_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include "acado_common.h"
#include <std_srvs/Empty.h>

#include <nmpc_controller/mpc_queue.h>
#include "nmpc_controller/mpc_wrapper.h"
#include "nmpc_controller/common.h"

namespace mav_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3f kDefaultPositionWeights = Eigen::Vector3f(6, 6, 6);
static const Eigen::Vector3f kDefaultVelocityWeights = Eigen::Vector3f(4.7, 4.7, 4.7);
static const Eigen::Vector3f kDefaultAttitudeWeights = Eigen::Vector3f(3, 3, 0.035);
static const Eigen::Vector3f kDefaultAngularRateWeights = Eigen::Vector3f(0.52, 0.52, 0.025);
static const float kDefaultThrustWeight = 1;
static const Eigen::Vector3f kDefaultTauWeight = Eigen::Vector3f(0.52, 0.52, 0.025);
static const float kDefaultMinThrust = 0.0;
static const float kDefaultMaxThrust = 30.0;
static const Eigen::Vector3f kDefaultMaxTau = Eigen::Vector3f(3,3,3);
static const float kDefaultSamplingTime = 0.1;
static const float kDefaultPredictionSamplingTime = 2.0;

class NonlinearMpcControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  NonlinearMpcControllerParameters()
      : q_position_(kDefaultPositionWeights),
        q_velocity_(kDefaultVelocityWeights),
        q_attitude_(kDefaultAttitudeWeights),
        q_angular_rate_(kDefaultAngularRateWeights),
        r_thrust_(kDefaultThrustWeight),
        r_tau_(kDefaultTauWeight),
        min_thrust_(kDefaultMinThrust),
        max_thrust_(kDefaultMaxThrust),
        max_tau_(kDefaultMaxTau),
        sampling_time_(kDefaultSamplingTime),
        prediction_sampling_time_(kDefaultPredictionSamplingTime) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;

  Eigen::Vector3f q_position_;
  Eigen::Vector3f q_velocity_;
  Eigen::Vector3f q_attitude_;
  Eigen::Vector3f q_angular_rate_;
  float r_thrust_;
  Eigen::Vector3f r_tau_;

  float min_thrust_;
  float max_thrust_;
  Eigen::Vector3f max_tau_;
  
  float sampling_time_;
  float prediction_sampling_time_;

  RotorConfiguration rotor_configuration_;
};  

class NonlinearMpcController
{
 public:
  NonlinearMpcController();
  ~NonlinearMpcController();

  NonlinearMpcControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  void setOdometry(const mav_msgs::EigenOdometry& odometry);
  void updateControlCommand();//for controller node

  void setCommandTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);//for controller node
  void setCommandTrajectory(const mav_msgs::EigenTrajectoryPointDeque& command_trajectory);//for controller node
  
  void getControlCommand(Eigen::Vector4d& torque_thrust){torque_thrust = command_torque_thrust_;}
    
  void initializeParameters();
  bool controller_active_{false};
  void CalculateRotorVelocities(const Eigen::Vector4d& torque_thrust, Eigen::VectorXd* rotor_velocities) const; 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  Eigen::Vector3f g_;
  Eigen::MatrixX4d torque_thrust_to_rotor_velocities_;
  MpcWrapper mpc_wrapper_;//000000000000000000000
  std::thread preparation_thread_;//00000000000000000
  MPCQueue mpc_queue_;//store trajectory into a deque
  mav_msgs::EigenTrajectoryPointDeque command_trajectory_;

  bool solve_from_scratch_{true};//0000000000000000000
  Eigen::Matrix<float, kStateSize, 1> est_state_;//00000000000000000

  Eigen::Matrix<float, kStateSize, kSamples + 1> reference_states_;//000000000000000000
  Eigen::Matrix<float, kInputSize, kSamples + 1> reference_inputs_;//000000000000000000

  //direct computed results:predicted inputs and predicted_states
  Eigen::Matrix<float, kStateSize, kSamples + 1> predicted_states_;//000000000000000000
  Eigen::Matrix<float, kInputSize, kSamples> predicted_inputs_;//000000000000000000000
  
  // commands
  Eigen::Vector4d command_torque_thrust_;//predicted_inputs

  // sampling time parameters

  bool initialized_parameters_{false};

  // controller weights
  Eigen::Matrix<float, kCostSize, kCostSize> Q_;//need to be initialized in initializeParameters
  Eigen::Matrix<float, kInputSize, kInputSize> R_;//need to be initialized in initializeParameters
  
  // reference states queue
  Vector3fDeque position_ref_, velocity_ref_, acc_ref_;
  std::deque<float> yaw_ref_, yaw_rate_ref_, yaw_acc_ref_;

  void preparationThread();
  void setCommandFromPredictedResults();

  void setLimitsAndWeights();
  bool limits_weights_set_successful_{false};
  void setReferenceStates();
  void setReferenceInputs();
};

}

#endif /* INCLUDE_NMPC_CONTROLLER_NONLINEAR_MPC_H_ */
