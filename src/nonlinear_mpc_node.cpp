#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <nmpc_controller/nonlinear_mpc_node.h>
#include "nmpc_controller/parameters_ros.h"

namespace mav_control {

NonLinearMpcControllerNode::NonLinearMpcControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &NonLinearMpcControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &NonLinearMpcControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
  
  torque_thrust_pub_ = nh_.advertise<mav_msgs::TorqueThrust>("torque_thrust",1);

  current_reference_inputs_pub_ = nh_.advertise<mav_msgs::TorqueThrust>("current_reference_inputs",1);

  current_reference_states_pub_ = nh_.advertise<nav_msgs::Odometry>("current_reference_states",1);

  timer_ = nh_.createTimer(ros::Duration(0.01), &NonLinearMpcControllerNode::TimedCommandCallback, this,
                                  false, false);
}

NonLinearMpcControllerNode::~NonLinearMpcControllerNode() { }

void NonLinearMpcControllerNode::InitializeParams() {
  GetRosParameter(private_nh_, "q_position/x",
                  nonlinear_mpc_.controller_parameters_.q_position_.x(),
                  &nonlinear_mpc_.controller_parameters_.q_position_.x());
  GetRosParameter(private_nh_, "q_position/y",
                  nonlinear_mpc_.controller_parameters_.q_position_.y(),
                  &nonlinear_mpc_.controller_parameters_.q_position_.y());
  GetRosParameter(private_nh_, "q_position/z",
                  nonlinear_mpc_.controller_parameters_.q_position_.z(),
                  &nonlinear_mpc_.controller_parameters_.q_position_.z());

  GetRosParameter(private_nh_, "q_velocity/x",
                  nonlinear_mpc_.controller_parameters_.q_velocity_.x(),
                  &nonlinear_mpc_.controller_parameters_.q_velocity_.x());
  GetRosParameter(private_nh_, "q_velocity/y",
                  nonlinear_mpc_.controller_parameters_.q_velocity_.y(),
                  &nonlinear_mpc_.controller_parameters_.q_velocity_.y());
  GetRosParameter(private_nh_, "q_velocity/z",
                  nonlinear_mpc_.controller_parameters_.q_velocity_.z(),
                  &nonlinear_mpc_.controller_parameters_.q_velocity_.z());

  GetRosParameter(private_nh_, "q_attitude/x",
                  nonlinear_mpc_.controller_parameters_.q_attitude_.x(),
                  &nonlinear_mpc_.controller_parameters_.q_attitude_.x());
  GetRosParameter(private_nh_, "q_attitude/y",
                  nonlinear_mpc_.controller_parameters_.q_attitude_.y(),
                  &nonlinear_mpc_.controller_parameters_.q_attitude_.y());
  GetRosParameter(private_nh_, "q_attitude/z",
                  nonlinear_mpc_.controller_parameters_.q_attitude_.z(),
                  &nonlinear_mpc_.controller_parameters_.q_attitude_.z());

  GetRosParameter(private_nh_, "q_angular_rate/x",
                  nonlinear_mpc_.controller_parameters_.q_angular_rate_.x(),
                  &nonlinear_mpc_.controller_parameters_.q_angular_rate_.x());
  GetRosParameter(private_nh_, "q_angular_rate/y",
                  nonlinear_mpc_.controller_parameters_.q_angular_rate_.y(),
                  &nonlinear_mpc_.controller_parameters_.q_angular_rate_.y());
  GetRosParameter(private_nh_, "q_angular_rate/z",
                  nonlinear_mpc_.controller_parameters_.q_angular_rate_.z(),
                  &nonlinear_mpc_.controller_parameters_.q_angular_rate_.z());
   
  GetRosParameter(private_nh_, "r_thrust",
                  nonlinear_mpc_.controller_parameters_.r_thrust_,
                  &nonlinear_mpc_.controller_parameters_.r_thrust_);
  GetRosParameter(private_nh_, "r_tau/x",
                  nonlinear_mpc_.controller_parameters_.r_tau_.x(),
                  &nonlinear_mpc_.controller_parameters_.r_tau_.x());
  GetRosParameter(private_nh_, "r_tau/y",
                  nonlinear_mpc_.controller_parameters_.r_tau_.y(),
                  &nonlinear_mpc_.controller_parameters_.r_tau_.y());
  GetRosParameter(private_nh_, "r_tau/z",
                  nonlinear_mpc_.controller_parameters_.r_tau_.z(),
                  &nonlinear_mpc_.controller_parameters_.r_tau_.z());

  GetRosParameter(private_nh_, "max_thrust",
                  nonlinear_mpc_.controller_parameters_.max_thrust_,
                  &nonlinear_mpc_.controller_parameters_.max_thrust_);
  GetRosParameter(private_nh_, "min_thrust",
                  nonlinear_mpc_.controller_parameters_.min_thrust_,
                  &nonlinear_mpc_.controller_parameters_.min_thrust_);

  GetRosParameter(private_nh_, "max_tau/x",
                  nonlinear_mpc_.controller_parameters_.max_tau_.x(),
                  &nonlinear_mpc_.controller_parameters_.max_tau_.x());
  GetRosParameter(private_nh_, "max_tau/y",
                  nonlinear_mpc_.controller_parameters_.max_tau_.y(),
                  &nonlinear_mpc_.controller_parameters_.max_tau_.y());
  GetRosParameter(private_nh_, "max_tau/z",
                  nonlinear_mpc_.controller_parameters_.max_tau_.z(),
                  &nonlinear_mpc_.controller_parameters_.max_tau_.z());

  GetRosParameter(private_nh_, "queue_dt",
                  nonlinear_mpc_.controller_parameters_.queue_dt_,
                  &nonlinear_mpc_.controller_parameters_.queue_dt_);
  GetRosParameter(private_nh_, "prediction_sampling_time",
                  nonlinear_mpc_.controller_parameters_.prediction_sampling_time_,
                  &nonlinear_mpc_.controller_parameters_.prediction_sampling_time_);

  GetVehicleParameters(private_nh_, &nonlinear_mpc_.vehicle_parameters_);
  nonlinear_mpc_.initializeParameters();
}

void NonLinearMpcControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  mav_msgs::EigenTrajectoryPointDeque command_trajectory;
  mav_msgs::eigenTrajectoryPointDequeFromMsg(*msg, &command_trajectory);
  nonlinear_mpc_.setCommandTrajectory(command_trajectory);
  if(!nonlinear_mpc_.controller_active_){
    timer_.start();
    nonlinear_mpc_.controller_active_ = true;
    std::cout << "controller is active now"<< std::endl;
  }
}

void NonLinearMpcControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  Eigen::Vector4d torque_thrust;
  if(!nonlinear_mpc_.controller_active_){
    torque_thrust = Eigen::Vector4d::Zero(torque_thrust.rows());
  }
  else{
    nonlinear_mpc_.updateControlCommand();//execute controller
    // std::cout << "if you can see this message, then it didnt fail here" << std::endl;
    nonlinear_mpc_.getControlCommand(torque_thrust);
  }
  
  Eigen::VectorXd ref_rotor_velocities;
  nonlinear_mpc_.CalculateRotorVelocities(torque_thrust, &ref_rotor_velocities);

  //publish motor speed message
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = ros::Time::now();
  motor_velocity_reference_pub_.publish(actuator_msg); 
  // publish torque_thrust message 

  mav_msgs::TorqueThrust torque_thrust_msg{};
  torque_thrust_msg.header = actuator_msg->header;
  torque_thrust_msg.thrust.x = 0;
  torque_thrust_msg.thrust.y = 0;
  torque_thrust_msg.thrust.z = torque_thrust(3);
  torque_thrust_msg.torque.x = torque_thrust(0);
  torque_thrust_msg.torque.y = torque_thrust(1);
  torque_thrust_msg.torque.z = torque_thrust(2);
  torque_thrust_pub_.publish(torque_thrust_msg);

  nav_msgs::Odometry current_reference_states_msg{};
  nonlinear_mpc_.getCurrentReferenceStates(current_reference_states_msg);
  current_reference_states_pub_.publish(current_reference_states_msg);

  nonlinear_mpc_.getCurrentReferenceInputs(torque_thrust_msg);
  current_reference_inputs_pub_.publish(torque_thrust_msg);
}

void NonLinearMpcControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  ROS_INFO_ONCE("LeePositionController got first odometry message.");
  mav_msgs::EigenOdometry odometry;  
  odometry.position_W = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry.orientation_W_B = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry.velocity_B = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry.angular_velocity_B = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
  nonlinear_mpc_.setOdometry(odometry);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nonlinear_mpc_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  mav_control::NonLinearMpcControllerNode nonlinear_mpc_controller_node(nh, private_nh);
  ros::spin();

  return 0;
}