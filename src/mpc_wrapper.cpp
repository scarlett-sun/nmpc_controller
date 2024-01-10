#include "nmpc_controller/mpc_wrapper.h"


namespace mav_control {

// Default Constructor.
MpcWrapper::MpcWrapper()
{
  // Clear solver memory.
  memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  memset(&acadoVariables, 0, sizeof( acadoVariables ));
  
  // Initialize the solver.
  acado_initializeSolver();//necessary to write here

  // Iinitialize AcadoVariables.
  initializeAcadoVariables();

  // Initialize solver.
  acado_initializeNodesByForwardSimulation();
  acado_preparationStep();
  acado_is_prepared_ = true;
}

// Constructor with cost matrices as arguments.
MpcWrapper::MpcWrapper(
  const Eigen::Ref<const Eigen::Matrix<float, kCostSize, kCostSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<float, kInputSize, kInputSize>> R)
{
  setCosts(Q, R);
  MpcWrapper();
}

// Set cost matrices with optional scaling.
bool MpcWrapper::setCosts(
  const Eigen::Ref<const Eigen::Matrix<float, kCostSize, kCostSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<float, kInputSize, kInputSize>> R,
  const float state_cost_scaling, const float input_cost_scaling)
{
  if(state_cost_scaling < 0.0 || input_cost_scaling < 0.0 )
  {
    ROS_ERROR("MPC: Cost scaling is wrong, must be non-negative!");
    return false;
  }
  W_.block(0, 0, kCostSize, kCostSize) = Q;
  W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
  WN_ = W_.block(0, 0, kCostSize, kCostSize);

  float state_scale{1.0};
  float input_scale{1.0};
  for(int i=0; i<kSamples; i++)
  { 
    state_scale = exp(- float(i)/float(kSamples)
      * float(state_cost_scaling));
    input_scale = exp(- float(i)/float(kSamples)
      * float(input_cost_scaling));
    acado_W_.block(0, i*kRefSize, kCostSize, kCostSize) =
      W_.block(0, 0, kCostSize, kCostSize).template cast<float>()
      * state_scale;
    acado_W_.block(kCostSize, i*kRefSize+kCostSize, kInputSize, kInputSize) =
      W_.block(kCostSize, kCostSize, kInputSize, kInputSize
        ).template cast<float>() * input_scale;
  } 
  acado_W_end_ = WN_.template cast<float>() * state_scale;

  return true;
}

// Set the input limits.
bool MpcWrapper::setLimits(const float& min_thrust, const float& max_thrust, const float& max_taux, const float& max_tauy, const float& max_tauz)
{
  if(min_thrust <= 0.0 || min_thrust > max_thrust)
  {
    ROS_ERROR("MPC: Minimal thrust is not set properly, not changed.");
    return false;
  }

  if(max_thrust <= 0.0 || min_thrust > max_thrust)
  {
    ROS_ERROR("MPC: Maximal thrust is not set properly, not changed.");
    return false;
  }

  if(max_taux <= 0.0)
  {
    ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
    return false;
  }

  if(max_tauy <= 0.0)
  {
    ROS_ERROR("MPC: Maximal yaw-rate is not set properly, not changed.");
    return false;
  }

  if(max_tauz <= 0.0)
  {
    ROS_ERROR("MPC: Maximal yaw-rate is not set properly, not changed.");
    return false;
  }

  // Set input boundaries.
  Eigen::Matrix<float, 4, 1> lower_bounds = Eigen::Matrix<float, 4, 1>::Zero();
  Eigen::Matrix<float, 4, 1> upper_bounds = Eigen::Matrix<float, 4, 1>::Zero();
  lower_bounds << -max_taux, -max_tauy, -max_tauz, min_thrust;
  upper_bounds <<  max_taux,  max_tauy,  max_tauz, max_thrust;

  acado_lower_bounds_ =
    lower_bounds.replicate(1, kSamples).template cast<float>();

  acado_upper_bounds_ =
    upper_bounds.replicate(1, kSamples).template cast<float>();
  return true;
}

void MpcWrapper::initializeAcadoVariables(){
  // Initialize states x and xN and input u.
  acado_initial_state_.setZero();
  acado_states_.setZero();
  acado_inputs_.setZero();

  // Initialize references y and yN.
  acado_reference_states_.setZero();
  acado_reference_end_state_.setZero();
  
  // Initialize weight
  W_.setZero();
  WN_.setZero();
  acado_W_ = W_.replicate(1, kSamples).template cast<float>();
  acado_W_end_ = WN_;

  // Initializa bounds
  acado_lower_bounds_.setZero();
  acado_upper_bounds_.setZero();
}

// // Set a reference pose.
// bool MpcWrapper::setReferencePose(
//   const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state)
// {
//   acado_reference_states_.block(0, 0, kStateSize, kSamples) =
//     state.replicate(1, kSamples).template cast<float>();

//   acado_reference_states_.block(kStateSize, 0, kCostSize-kStateSize, kSamples) =
//     Eigen::Matrix<float, kCostSize-kStateSize, kSamples>::Zero();

//   acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
//     kHoverInput_.replicate(1, kSamples);

//   acado_reference_end_state_.segment(0, kStateSize) =
//     state.template cast<float>();

//   acado_reference_end_state_.segment(kStateSize, kCostSize-kStateSize) =
//     Eigen::Matrix<float, kCostSize-kStateSize, 1>::Zero();

//   acado_initializeNodesByForwardSimulation();
//   return true;
// }

// Set a reference trajectory.
bool MpcWrapper::setReferences(
  const Eigen::Ref<const Eigen::Matrix<float, kStateSize, kSamples+1>> states,
  const Eigen::Ref<const Eigen::Matrix<float, kInputSize, kSamples+1>> inputs)
{
  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
    y(const_cast<float*>(acadoVariables.y));

  acado_reference_states_.block(0, 0, kStateSize, kSamples) =
    states.block(0, 0, kStateSize, kSamples).template cast<float>();

  acado_reference_states_.block(kStateSize, 0, kCostSize-kStateSize, kSamples) =
    Eigen::Matrix<float, kCostSize-kStateSize, kSamples>::Zero();

  acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
    inputs.block(0, 0, kInputSize, kSamples).template cast<float>();

  acado_reference_end_state_.segment(0, kStateSize) =
    states.col(kSamples).template cast<float>();
  acado_reference_end_state_.segment(kStateSize, kCostSize-kStateSize) =
    Eigen::Matrix<float, kCostSize-kStateSize, 1>::Zero();

  return true;
}

// Reset states and inputs and calculate new solution.
bool MpcWrapper::solve(
  const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state)
{
  acado_states_ = state.replicate(1, kSamples+1).template cast<float>();
  acado_inputs_.setZero();
  // acado_inputs_ = kHoverInput_.replicate(1, kSamples);

  return update(state);
}


// Calculate new solution from last known solution.
bool MpcWrapper::update(
  const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state)
{
  if(!acado_is_prepared_)
  {
    ROS_WARN("MPC: Solver was triggered without preparation, abort!");
    return false;
  }

  setInitialState(state);
  
  //先prepare，再feedback，执行一个控制周期。
  // Perform feedback step and reset preparation check.
  acado_feedbackStep();//理论上，执行完这一句就是求完了一轮了。acado_states_和acado_inputs_应该都更新了求解后的内容了。
  acado_is_prepared_ = false;

  //执行prepare
  return true;
}

// Prepare the solver.
// Must be triggered between iterations if not done in the update function.S
bool MpcWrapper::prepare()
{
  acado_preparationStep();
  acado_is_prepared_ = true;
  return true;
}

// Get a specific state.
void MpcWrapper::getState(const int node_index,
    Eigen::Ref<Eigen::Matrix<float, kStateSize, 1>> return_state)
{
  return_state = acado_states_.col(node_index).cast<float>();
}

// Get all states.
void MpcWrapper::getStates(
    Eigen::Ref<Eigen::Matrix<float, kStateSize, kSamples+1>> return_states)
{
  return_states = acado_states_.cast<float>();
}

// Get a specific input.
void MpcWrapper::getInput(const int node_index,
    Eigen::Ref<Eigen::Matrix<float, kInputSize, 1>> return_input)
{
  return_input = acado_inputs_.col(node_index).cast<float>();
}

// Get all inputs.
void MpcWrapper::getInputs(
    Eigen::Ref<Eigen::Matrix<float, kInputSize, kSamples>> return_inputs)
{
  return_inputs = acado_inputs_.cast<float>();
}

} // namespace rpg_mpc

