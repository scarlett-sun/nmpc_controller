/*
 Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland

 You can contact the author at <mina.kamel@mavt.ethz.ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_H_
#define INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <stdio.h>
#include <mav_control_interface/mpc_queue.h>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <std_srvs/Empty.h>
#include <lapacke.h>

namespace mav_control {

class NonlinearMpcController
{
 public:
  NonlinearMpcController();
  ~NonlinearMpcController();

  // // get reference and predicted state
  // bool getCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const;
  // bool getCurrentReference(mav_msgs::EigenTrajectoryPointDeque* reference) const;
  // bool getPredictedState(mav_msgs::EigenTrajectoryPointDeque* predicted_state) const;

  // set odom and commands
  void setLimits();
  void setWeights();
  void setEstimatedState();
  void setReference();
  void updateControlCommand();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  MpcWrapper mpc_wrapper_;
  std::thread preparation_thread_;
  // constants
  static constexpr double kGravity = 9.8066;

  bool solve_from_scratch_;
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;

  // sampling time parameters
  void initializeParameters();
  bool initialized_parameters_;

  // sampling time parameters
  double sampling_time_;
  double prediction_sampling_time_;

  // system model parameters
  double mass_;
  double Jxx_;
  double Jyy_;
  double Jzz_;

  // control input limits
  double taux_limit_;
  double tauy_limit_;
  double tauz_limit_;
  double thrust_min_;
  double thrust_max_;

  // reference queue
  MPCQueue mpc_queue_;
  Vector3dDeque position_ref_, velocity_ref_, acceleration_ref_;
  Vector3dDeque attitude_ref_, angular_velocity_ref_, angular_acceleration_ref_;

  // commands
  Eigen::Vector4d command_torque_thrust_;

  // debug info
  bool verbose_;
  double solve_time_average_;

  // most recent odometry information
  mav_msgs::EigenOdometry odometry_;
  bool received_first_odometry_;

  // solve continuous time Riccati equation
  Eigen::MatrixXd solveCARE(Eigen::MatrixXd Q, Eigen::MatrixXd R);

  void preparationThread();
};

}

#endif /* INCLUDE_MAV_NONLINEAR_MPC_NONLINEAR_MPC_H_ */
