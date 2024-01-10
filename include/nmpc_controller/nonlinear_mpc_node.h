#ifndef INCLUDE_NMPC_CONTROLLER_NONLINEAR_MPC_NODE_H_
#define INCLUDE_NMPC_CONTROLLER_NONLINEAR_MPC_NODE_H_

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

//ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

//ros msgs
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/Status.h>
#include <mav_msgs/TorqueThrust.h>

#include <nmpc_controller/nonlinear_mpc.h>
// #include <mav_control_interface/position_controller_interface.h>

namespace mav_control {

class NonLinearMpcControllerNode
{
 public:
  NonLinearMpcControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~NonLinearMpcControllerNode();

  void InitializeParams();//related to mpc parameter details

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  NonlinearMpcController nonlinear_mpc_;


    // subscribers
  // ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  // ros::Subscriber cmd_pose_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher motor_velocity_reference_pub_;
  ros::Publisher torque_thrust_pub_;

  ros::Publisher current_reference_states_pub_;
  ros::Publisher current_reference_inputs_pub_;

  ros::Timer timer_;//100Hz

  void TimedCommandCallback(const ros::TimerEvent& e);// related to mpc output details

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
};

}


#endif /* INCLUDE_NMPC_CONTROLLER_NONLINEAR_MPC_NODE_H_ */
