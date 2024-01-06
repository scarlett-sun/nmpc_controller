#include <tf/transform_datatypes.h>

#include <nmpc_controller/nonlinear_mpc.h>

namespace mav_control {

constexpr double NonlinearMpcController::kGravity;

NonlinearMpcController::NonlinearMpcController(){
  //初始化，初始位置，参考状态，参考输入，预测状态，预测输入
  //初始化，Q和R的权重矩阵（Q和R是私有成员）
  //初始化，limits（limitis是私有成员）
  //权重和limits初始化完毕后
}
NonlinearMpcController::~NonlinearMpcController(){}

NonlinearModelPredictiveControl::NonlinearModelPredictiveControl(const ros::NodeHandle& nh,
                                                                 const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_parameters_(false),//参数初始化
      mpc_queue_(nh, private_nh, ACADO_N+1),
      command_torque_thrust_(0, 0, 0, 0),
      verbose_(false),
      solve_time_average_(0),//平均求解时间
      received_first_odometry_(false)//接收到第一个状态
{

  acado_initializeSolver();//首先初始化acado求解器，是模型预测控制器的一部分

  W_.setZero();
  WN_.setZero();//N是什么意思

  input_.setZero();
  state_.setZero();
  reference_.setZero();
  referenceN_.setZero();

  initializeParameters();

  mpc_queue_.initializeQueue(sampling_time_, prediction_sampling_time_);

}


void NonlinearModelPredictiveControl::initializeParameters()
{
  //Get parameters from RosParam server
  private_nh_.param<bool>("verbose", verbose_, false);

  if (!private_nh_.getParam("mass", mass_)) {
    ROS_ERROR("mass in nonlinear MPC controller is not loaded from ros parameter "
              "server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in nonlinear MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    ROS_ERROR("prediction_sampling_time in nonlinear MPC is not loaded from ros parameter server");
    abort();
  }

  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();

  if (verbose_) {
    std::cout << "acado online data: " << std::endl << acado_online_data_ << std::endl;
  }

  initialized_parameters_ = true;
  ROS_INFO("Nonlinear MPC: initialized correctly");
}

void NonlinearModelPredictiveControl::applyParameters()
{
  W_.block(0, 0, 3, 3) = q_position_.asDiagonal();
  W_.block(3, 3, 3, 3) = q_attitude_.asDiagonal();
  W_.block(6, 6, 3, 3) = q_velocity_.asDiagonal();
  W_.block(9, 9, 3, 3) = q_omega_.asDiagonal();
  W_.block(12, 12, 4, 4) = r_command_.asDiagonal();

  WN_ = solveCARE((Eigen::VectorXd(6) << q_position_, q_velocity_).finished().asDiagonal(),
                  r_command_.asDiagonal());//?

  for (size_t i = 0; i < ACADO_N; ++i) {
    acadoWorkspace.lb[3 * i] = -taux_limit_;       // min roll
    acadoWorkspace.lb[3 * i + 1] = -tauy_limit_;  // min pitch
    acadoWorkspace.lb[3 * i + 2] = thrust_min_;    // min thrust
    acadoWorkspace.ub[3 * i] = taux_limit_;        // max roll
    acadoWorkspace.ub[3 * i + 1] = tauy_limit_;   // max pitch
    acadoWorkspace.ub[3 * i + 2] = thrust_max_;    // max thrust
  }

  if (verbose_) {
    std::cout << "q_position_: " << q_position_.transpose() << std::endl;
    std::cout << "q_velocity_: " << q_velocity_.transpose() << std::endl;
    std::cout << "r_command_: " << r_command_.transpose() << std::endl;
    std::cout << "W_N = \n" << WN_ << std::endl;
  }
}

void NonlinearModelPredictiveControl::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  static mav_msgs::EigenOdometry previous_odometry = odometry;

  if (!received_first_odometry_) {
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(&euler_angles);

    Eigen::VectorXd x0(ACADO_NX);

    x0 << odometry.position_W,  euler_angles, odometry.getVelocityWorld(), odometry.angular_velocity_B;

    initializeAcadoSolver(x0);

    received_first_odometry_ = true;
  }

  if (odometry.position_W.allFinite() == false) {
    odometry_.position_W = previous_odometry.position_W;
    ROS_WARN("Odometry.position has a non finite element");
  } else {
    odometry_.position_W = odometry.position_W;
    previous_odometry.position_W = odometry.position_W;
  }

  if (odometry.velocity_B.allFinite() == false) {
    odometry_.velocity_B = previous_odometry.velocity_B;
    ROS_WARN("Odometry.velocity has a non finite element");
  } else {
    odometry_.velocity_B = odometry.velocity_B;
    previous_odometry.velocity_B = odometry.velocity_B;
  }

  if (odometry.angular_velocity_B.allFinite() == false) {
    odometry_.angular_velocity_B = previous_odometry.angular_velocity_B;
    ROS_WARN("Odometry.angular_velocity has a non finite element");
  } else {
    odometry_.angular_velocity_B = odometry.angular_velocity_B;
    previous_odometry.angular_velocity_B = odometry.angular_velocity_B;
  }

  odometry_.orientation_W_B = odometry.orientation_W_B;
  odometry_.timestamp_ns = odometry.timestamp_ns;
  previous_odometry.orientation_W_B = odometry.orientation_W_B;
}

void NonlinearModelPredictiveControl::setCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  mpc_queue_.insertReference(command_trajectory);
}

void NonlinearModelPredictiveControl::setCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory)
{
  int array_size = command_trajectory.size();
  if (array_size < 1)
    return;

  mpc_queue_.insertReferenceTrajectory(command_trajectory);
}

void NonlinearModelPredictiveControl::initializeAcadoSolver(Eigen::VectorXd x0)
{
  static_assert(ACADO_N == 20,"acado_n error");
  for (int i = 0; i < ACADO_N + 1; i++) {
    state_.block(i, 0, 1, ACADO_NX) << x0.transpose();
  }

  Eigen::Map<Eigen::Matrix<double, ACADO_NX, ACADO_N + 1>>(const_cast<double*>(acadoVariables.x)) =
      state_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NU, ACADO_N>>(const_cast<double*>(acadoVariables.u)) =
      input_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
      reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
      referenceN_.transpose();
}

void NonlinearModelPredictiveControl::calculateTorqueThrustCommand(
    Eigen::Vector4d* ref_torque_thrust)
{
  assert(ref_torque_thrust != nullptr);
  assert(initialized_parameters_ == true);
  ros::WallTime starting_time = ros::WallTime::now();

  Eigen::Matrix<double, ACADO_NX, 1> x_0;// initial state

  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(&current_rpy);

  mpc_queue_.updateQueue();//need to be changed 
  mpc_queue_.getQueue(position_ref_, velocity_ref_, acceleration_ref_, yaw_ref_, yaw_rate_ref_);//need to be changed to include torque and thrust


  for (size_t i = 0; i < ACADO_N; i++) {//update reference ,need to be changed
    Eigen::Vector3d acceleration_ref_B = odometry_.orientation_W_B.toRotationMatrix().transpose() * acceleration_ref_[i];

    reference_.block(i, 0, 1, ACADO_NY) << position_ref_[i].transpose(), velocity_ref_[i].transpose(), acceleration_ref_[i].z();
  }
  referenceN_ << position_ref_[ACADO_N].transpose(), velocity_ref_[ACADO_N].transpose();// and rotation ref

  x_0 << odometry_.getVelocityWorld(), current_rpy, odometry_.position_W;

  Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x_0;
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
      reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
      referenceN_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();

  ros::WallTime time_before_solving = ros::WallTime::now();

  acado_preparationStep();

  int acado_status = acado_feedbackStep();

  solve_time_average_ += (ros::WallTime::now() - time_before_solving).toSec() * 1000.0;

  double taux_ref = acadoVariables.u[0];
  double tauy_ref = acadoVariables.u[1];
  double tauz_ref = acadoVariables.u[2];

  if (std::isnan(roll_ref) || std::isnan(pitch_ref) || std::isnan(thrust_ref)
      || acado_status != 0) {
    ROS_WARN_STREAM("Nonlinear MPC: Solver failed with status: " << acado_status);
    ROS_WARN("reinitializing...");
    initializeAcadoSolver (x_0);
    *ref_torque_thrust << 0, 0, 0, kGravity * mass_;
    return;
  }

  command_torque_thrust_ << taux_ref, tauy_ref, tauz_ref_, thrust_ref;

  state_ = Eigen::Map<Eigen::Matrix<double, ACADO_N + 1, ACADO_NX, Eigen::RowMajor>>(
      acadoVariables.x);

  *ref_torque_thrust = Eigen::Vector4d(taux_ref, tauy_ref, tauz_ref, mass_ * thrust_ref);

  double diff_time = (ros::WallTime::now() - starting_time).toSec();

  if (verbose_) {
    static int counter = 0;
    if (counter > 100) {
      ROS_INFO_STREAM("average solve time: " << solve_time_average_ / counter << " ms");
      solve_time_average_ = 0.0;

      ROS_INFO_STREAM("Controller loop time : " << diff_time*1000.0 << " ms");

      ROS_INFO_STREAM(
          "roll ref: " << command_roll_pitch_yaw_thrust_(0) << "\t" << "pitch ref : \t" << command_roll_pitch_yaw_thrust_(1) << "\t" << "yaw ref : \t" << command_roll_pitch_yaw_thrust_(2) << "\t" << "thrust ref : \t" << command_roll_pitch_yaw_thrust_(3) << "\t" << "yawrate ref : \t" << yaw_rate_cmd);
      counter = 0;
    }
    counter++;
  }

}

bool NonlinearModelPredictiveControl::getCurrentReference(// needs to be changed, state reference and input reference
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);

  (*reference).position_W = position_ref_.front();
  (*reference).velocity_W = velocity_ref_.front();
  (*reference).acceleration_W = acceleration_ref_.front();
  (*reference).setFromYaw(yaw_ref_.front());

  return true;
}

bool NonlinearModelPredictiveControl::getCurrentReference(// needs to be changed
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);

  (*reference).clear();

  for (size_t i = 0; i < position_ref_.size(); i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = position_ref_.at(i);
    pnt.velocity_W = velocity_ref_.at(i);
    pnt.acceleration_W = acceleration_ref_.at(i);
    pnt.setFromYaw(yaw_ref_.at(i));
    (*reference).push_back(pnt);
  }
  return true;
}

bool NonlinearModelPredictiveControl::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);

  for (size_t i = 0; i < ACADO_N + 1; i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = state_.block(i, 6, 1, 3).transpose();
    pnt.velocity_W = state_.block(i, 0, 1, 3).transpose();

    tf::Quaternion tf_orientation;
    tf_orientation.setRPY(state_(i,3), state_(i,4), state_(i,5));
    pnt.orientation_W_B.x() = tf_orientation.x();
    pnt.orientation_W_B.y() = tf_orientation.y();
    pnt.orientation_W_B.z() = tf_orientation.z();
    pnt.orientation_W_B.w() = tf_orientation.w();
                
    pnt.time_from_start_ns = static_cast<int64_t>(i) *
                           static_cast<int64_t>(sampling_time_ * 1000000000.0);
    pnt.timestamp_ns = odometry_.timestamp_ns + pnt.time_from_start_ns;

    (*predicted_state).push_back(pnt);
  }

  return true;
}

}
