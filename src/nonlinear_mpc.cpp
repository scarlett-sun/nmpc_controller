#include <tf/transform_datatypes.h>

#include <nmpc_controller/nonlinear_mpc.h>

namespace mav_control {

NonlinearMpcController::NonlinearMpcController():mpc_queue_(ACADO_N+1){
  g_ = Eigen::Vector3f(0,0,vehicle_parameters_.gravity_);
}

NonlinearMpcController::~NonlinearMpcController(){}

/*Set Q_ and R_*/
void NonlinearMpcController::initializeParameters(){
  Q_ = (Eigen::Matrix<float, kCostSize, 1>() <<
    controller_parameters_.q_position_,//position
    controller_parameters_.q_attitude_,//attitude
    controller_parameters_.q_velocity_,//velocity
    controller_parameters_.q_angular_rate_).finished().asDiagonal();//初始化权重矩阵，这里的权重数值可能得改
  R_ = (Eigen::Matrix<float, kInputSize, 1>() <<
      controller_parameters_.r_tau_,controller_parameters_.r_thrust_).finished().asDiagonal();

  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));

  torque_thrust_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  torque_thrust_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();
  initialized_parameters_ = true;
  
  setLimitsAndWeights();
  mpc_queue_.initializeQueue(controller_parameters_.sampling_time_,controller_parameters_.prediction_sampling_time_);
  preparation_thread_ = std::thread(&MpcWrapper::prepare, mpc_wrapper_);

}

void NonlinearMpcController::getCurrentReferenceStates(nav_msgs::Odometry& current_reference_states){
  geometry_msgs::Vector3 vec3;
  geometry_msgs::Point vec_p;
  geometry_msgs::Quaternion quat;

  mav_msgs::pointEigenToMsg(position_ref_.front().cast<double>(),&vec_p);
  current_reference_states.pose.pose.position = vec_p;
  
  mav_msgs::quaternionEigenToMsg(mav_msgs::quaternionFromYaw(yaw_ref_.front()),&quat);
  current_reference_states.pose.pose.orientation = quat;

  mav_msgs::vectorEigenToMsg(velocity_ref_.front().cast<double>(),&vec3);
  current_reference_states.twist.twist.linear = vec3;
  if(vec3.z>0.5){
    std::cout << "Something is wrong with the z veloctiy" <<std::endl;
    std::cout << vec3.z << std::endl;
  }

  mav_msgs::setAngularVelocityMsgFromYawRate(yaw_rate_ref_.front(),&vec3);
  current_reference_states.twist.twist.angular = vec3;
}

void NonlinearMpcController::getCurrentReferenceInputs(mav_msgs::TorqueThrust& current_reference_inputs){
  current_reference_inputs.thrust.x = 0;
  current_reference_inputs.thrust.y = 0;
  current_reference_inputs.thrust.z = thrust_ref_.front();
  current_reference_inputs.torque.x = torque_ref_.front().x();
  current_reference_inputs.torque.y = torque_ref_.front().y();
  current_reference_inputs.torque.z = torque_ref_.front().z();
}


void NonlinearMpcController::setLimitsAndWeights(){
  mpc_wrapper_.setCosts(Q_,R_);
  mpc_wrapper_.setLimits(controller_parameters_.min_thrust_,
                         controller_parameters_.max_thrust_,
                         controller_parameters_.max_tau_.x(),
                         controller_parameters_.max_tau_.y(),
                         controller_parameters_.max_tau_.z());
  limits_weights_set_successful_ = true;
  //如果cost和limit的ros参数已经设定好了（bool limits_weights_read_successful），那么就允许往acado设置limits和weights，设置完后，set bool limits_weights_set_successful=true
}

void NonlinearMpcController::updateControlCommand(){
  // ros::Time call_time = ros::Time::now();//用于时间戳，不确定
  if(!limits_weights_set_successful_){
    ROS_ERROR("limits and weights not set");
    return;
  }
  // std::cout << "point A" << std::endl;
  mpc_queue_.updateQueue();//getQueue函数有问题！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！1
  // std::cout << "point B" << std::endl;
  mpc_queue_.getQueue(
    position_ref_, velocity_ref_, acc_ref_, yaw_ref_, yaw_rate_ref_,yaw_acc_ref_);//need to be changed to include torque and thrust
  // std::cout << "point C" << std::endl;
  setReferenceStates();//把xxx_ref_转写成reference_states和reference_inputs_
  // std::cout << "point D" << std::endl;
  setReferenceInputs();
  // std::cout << "point E" << std::endl;
  preparation_thread_.join();
  // std::cout << "point F" << std::endl;
  mpc_wrapper_.setInitialState(est_state_);
  // std::cout << "point G" << std::endl;
  mpc_wrapper_.setReferences(reference_states_,reference_inputs_);
  // std::cout << "point H" << std::endl;
  //计算，暂时等同于从初始位置进行求解
  if (solve_from_scratch_) {
    ROS_INFO("Solving MPC with ground as initial guess.");
    mpc_wrapper_.solve(est_state_);
    // std::cout << "point I" << std::endl;
    solve_from_scratch_ = false;
  } else {
    mpc_wrapper_.update(est_state_);
    // std::cout << "point J" << std::endl;
  }

  // Start a thread to prepare for the next execution.
  preparation_thread_ = std::thread(&NonlinearMpcController::preparationThread, this);
  // std::cout << "point K" << std::endl;
  mpc_wrapper_.getInputs(predicted_inputs_);//换成消息，不要eigen，或者用结构体，包含时间戳
  // std::cout << "point L" << std::endl;
  setCommandFromPredictedResults();//这里得加上时间戳！！！！！！！！！！！！！！！！！！！！！！待办
}

// void NonlinearMpcController::setReferenceStates(){
//   //  Eigen::Matrix<float, kStateSize, kSamples + 1> reference_states_;
//   Eigen::VectorXf state;
//   int i = 0;
//   for (auto it = position_ref_.begin(); it != position_ref_.end(); ++it) {
//     state.segment(0,3) = *it;
//     state.segment(3,1).setZero();
//     state.segment(4,1).setZero();
//     state.segment(5,1) = yaw_ref_.front();
//     state.segment(6,3) = velocity_ref_.front();
//     state.segment(9,1).setZero();
//     state.segment(10,1).setZero();
//     state.segment(11,1) = yaw_rate_ref_.front();

//     yaw_ref_.pop_front();
//     velocity_ref_.pop_front();
//     yaw_rate_ref_.pop_front();

//     reference_states_.block(0,i,kStateSize,1) = state;
//     i++;
//   }
// }//注意是i还是i+1是；sample还是sample+1，要assert一下

void NonlinearMpcController::setReferenceStates() {
    // Check if all reference vectors have the same size
    if (position_ref_.size() != yaw_ref_.size() ||
        position_ref_.size() != velocity_ref_.size() ||
        position_ref_.size() != yaw_rate_ref_.size()) {
        // Handle error or throw an exception
        // std::cout << "Error: size not the same" << std::endl;
        return;
    }

    // Initialize state vector
    Eigen::VectorXf state(kStateSize);
    // std::cout << "position ref size: "<< position_ref_.size() <<std::endl;
    int i = 0;
    auto it = position_ref_.begin();
    while(i < position_ref_.size()){
      // Populate state vector segments
      state.segment(0, 3) = *it;
      state.segment(3, 1).setZero();
      state.segment(4, 1).setZero();
      state.segment(5, 1).setConstant(yaw_ref_.front());
      state.segment(6, 3) = velocity_ref_.front();
      state.segment(9, 1).setZero();
      state.segment(10, 1).setZero();
      state.segment(11, 1).setConstant(yaw_rate_ref_.front());

      // Remove front elements from reference vectors
      yaw_ref_.pop_front();
      velocity_ref_.pop_front();
      yaw_rate_ref_.pop_front();

      // std::cout << "i: "<< i << std::endl;
      // Assign state vector to reference_states_
      reference_states_.block(0, i, kStateSize, 1) = state;
      if(i<position_ref_.size()){
        i++;
        it++;
      }
    }

    // for (auto it = position_ref_.begin(); it != position_ref_.end(); ++it) {
    //   // Populate state vector segments
    //   state.segment(0, 3) = *it;
    //   state.segment(3, 1).setZero();
    //   state.segment(4, 1).setZero();
    //   state.segment(5, 1).setConstant(yaw_ref_.front());
    //   state.segment(6, 3) = velocity_ref_.front();
    //   state.segment(9, 1).setZero();
    //   state.segment(10, 1).setZero();
    //   state.segment(11, 1).setConstant(yaw_rate_ref_.front());

    //   // Remove front elements from reference vectors
    //   yaw_ref_.pop_front();
    //   velocity_ref_.pop_front();
    //   yaw_rate_ref_.pop_front();

    //   // Assign state vector to reference_states_
    //   reference_states_.block(0, i, kStateSize, 1) = state;
    //   i++;
    // }
    // std::cout << "i: "<< i << std::endl;
}

// void NonlinearMpcController::setReferenceInputs(){
//   Eigen::Matrix<float, kInputSize, 1> input;
//   for(int i = 0 ; i < (kSamples+1);i++){

//     input.segment(3,1) = vehicle_parameters_.mass_ * (acc_ref_.at(i) - g_);
//     Eigen::Vector3d omega_dot = Eigen::Vector3d(0,0,yaw_acc_ref_.at(i));
//     Eigen::Vector3d omega = Eigen::Vector3d(0,0,yaw_rate_ref_.at(i));
//     input.segment(0,3) = vehicle_parameters_.inertia_ * omega_dot + omega.cross(vehicle_parameters_.inertia_ * omega);
//     reference_inputs_.block(0,i,kInputSize,1) = input;
//   }
// }

void NonlinearMpcController::setReferenceInputs() {
    // Check if the size of the references matches the expected size
    if (acc_ref_.size() != kSamples + 1 ||
        yaw_acc_ref_.size() != kSamples + 1 ||
        yaw_rate_ref_.size() != kSamples + 1) {
        // Handle error or throw an exception
        return;
    }

    Eigen::Matrix<float, kInputSize, 1> input;

    for (int i = 0; i < (kSamples + 1); ++i) {
        // Compute input.segment(3,1)
        input.segment(3, 1) = vehicle_parameters_.mass_ * (acc_ref_.at(i) - g_);

        // Compute omega_dot and omega
        Eigen::Vector3f omega_dot = Eigen::Vector3f(0, 0, yaw_acc_ref_.at(i));
        Eigen::Vector3f omega = Eigen::Vector3f(0, 0, yaw_rate_ref_.at(i));

        // Compute input.segment(0,3)
        input.segment(0, 3) = vehicle_parameters_.inertia_.cast<float>() * omega_dot + omega.cross(vehicle_parameters_.inertia_.cast<float>() * omega);

        // Assign the computed input to reference_inputs_
        reference_inputs_.block(0, i, kInputSize, 1) = input;
        thrust_ref_.push_back(input(3, 0));
        torque_ref_.push_back(input.segment(0, 3));
    }
}




void NonlinearMpcController::setOdometry
  (const mav_msgs::EigenOdometry& odometry){
    Eigen::Matrix<float, kStateSize, 1> est_state;
    est_state.block(0,0,3,1) = odometry.position_W.cast<float>();
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(&euler_angles);
    est_state.block(3,0,3,1) = euler_angles.cast<float>();
    est_state.block(6,0,3,1) = odometry.getVelocityWorld().cast<float>();
    est_state.block(9,0,3,1) = odometry.angular_velocity_B.cast<float>();
  est_state_ = est_state;
}

// void NonlinearMpcController::setCommandFromPredictedResults(){
//   command_torque_thrust_[0] = predicted_inputs_[0][0];
//   command_torque_thrust_[1] = predicted_inputs_[1][0];
//   command_torque_thrust_[2] = predicted_inputs_[2][0];
//   command_torque_thrust_[3] = predicted_inputs_[3][0];
// }

void NonlinearMpcController::setCommandFromPredictedResults() {
    // Ensure that predicted_inputs_ has at least 4 columns
    if (predicted_inputs_.cols() < 4) {
        // Handle error or throw an exception
        return;
    }

    // Set command_torque_thrust_ values based on predicted_inputs_
    command_torque_thrust_[0] = predicted_inputs_.row(0)[0];
    command_torque_thrust_[1] = predicted_inputs_.row(1)[0];
    command_torque_thrust_[2] = predicted_inputs_.row(2)[0];
    command_torque_thrust_[3] = predicted_inputs_.row(3)[0];
}

void NonlinearMpcController::preparationThread() {
  const clock_t start = clock();

  mpc_wrapper_.prepare();

  // // Timing
  // const clock_t end = clock();
  // timing_preparation_ = 0.9 * timing_preparation_ +
  //                       0.1 * double(end - start) / CLOCKS_PER_SEC;//两项内容应该大致相等，这里只是取了一个加权值，好像叫滑动平均
}

void NonlinearMpcController::setCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  mpc_queue_.insertReference(command_trajectory);
}

void NonlinearMpcController::setCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory)
{
  command_trajectory_ = command_trajectory;
  int array_size = command_trajectory.size();
  if (array_size < 1)
    return;
  else if(array_size == 1){
    mpc_queue_.insertReference(command_trajectory.front());
  }
  else{
    mpc_queue_.insertReferenceTrajectory(command_trajectory);
    // std::cout << "The trajectory should be inserted"<< std::endl;
  }
}

void NonlinearMpcController::CalculateRotorVelocities(const Eigen::Vector4d& torque_thrust, Eigen::VectorXd* rotor_velocities) const {
  assert(rotor_velocities);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());

  *rotor_velocities = torque_thrust_to_rotor_velocities_ * torque_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

}