#pragma once

#include <Eigen/Eigen>
#include <ros/ros.h>

namespace mav_control {

#include "acado_auxiliary_functions.h"
#include "acado_common.h"

static constexpr int kSamples = ACADO_N;      // number of samples, 20
static constexpr int kStateSize = ACADO_NX;   // number of states, 12
static constexpr int kRefSize = ACADO_NY;     // number of reference states, 12+4=16
static constexpr int kEndRefSize = ACADO_NYN; // number of end reference states, 12
static constexpr int kInputSize = ACADO_NU;   // number of inputs, 4
static constexpr int kCostSize = ACADO_NY - ACADO_NU; // number of state costs, 12
static constexpr int kOdSize = ACADO_NOD;     // number of online data, 0，需要再编译一遍，暂时不需要online data

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

class MpcWrapper
{
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcWrapper();//默认构造
  MpcWrapper(
    const Eigen::Ref<const Eigen::Matrix<float, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<float, kInputSize, kInputSize>> R);//用权重矩阵构造
 
  /*设置权重*/
  bool setCosts(
    const Eigen::Ref<const Eigen::Matrix<float, kCostSize, kCostSize>> Q,
    const Eigen::Ref<const Eigen::Matrix<float, kInputSize, kInputSize>> R,
    const float state_cost_scaling = 0.0, const float input_cost_scaling = 0.0);//用权重矩阵构造，且便于调参
  
  /*设置约束*/
  bool setLimits(const float& min_thrust, const float& max_thrust, const float& max_taux, const float& max_tauy, const float& max_tauz);
  

  void initializeAcadoVariables();

  /*设置初始状态*/
  void setInitialState(const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> est_state){
    acado_initial_state_ = est_state.template cast<float>();
  }

  // /*设置参考位姿（一个轨迹点）*/
  // bool setReferencePose(const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state);
  
  /*设置参考轨迹（一系列轨迹点）*/
  bool setReferences(
    const Eigen::Ref<const Eigen::Matrix<float, kStateSize, kSamples+1>> states,
    const Eigen::Ref<const Eigen::Matrix<float, kInputSize, kSamples+1>> inputs);

  /*求解*/
  bool solve(const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state);

  /*更新*/
  bool update(const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state);
  
  /*准备*/
  bool prepare();
  
  /*获取单个（预测）状态*/
  void getState(const int node_index, Eigen::Ref<Eigen::Matrix<float, kStateSize, 1>> return_state);
  
  /*获取所有（预测）状态*/
  void getStates(Eigen::Ref<Eigen::Matrix<float, kStateSize, kSamples+1>> return_states);
  
  /*获取单个（预测）输入*/
  void getInput(const int node_index,
    Eigen::Ref<Eigen::Matrix<float, kInputSize, 1>> return_input);

  /*获取所有（预测）输入*/
  void getInputs(
    Eigen::Ref<Eigen::Matrix<float, kInputSize, kSamples>> return_input);
  
  /*获取MPC预测步长*/
  float getTimestep() { return dt_; }

 private:
  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
    acado_reference_states_{acadoVariables.y};//12+4=16

  Eigen::Map<Eigen::Matrix<float, kEndRefSize, 1, Eigen::ColMajor>>
    acado_reference_end_state_{acadoVariables.yN};//12

  Eigen::Map<Eigen::Matrix<float, kStateSize, 1, Eigen::ColMajor>>
    acado_initial_state_{acadoVariables.x0};//12

  Eigen::Map<Eigen::Matrix<float, kStateSize, kSamples+1, Eigen::ColMajor>>
    acado_states_{acadoVariables.x};//12*21

  Eigen::Map<Eigen::Matrix<float, kInputSize, kSamples, Eigen::ColMajor>>
    acado_inputs_{acadoVariables.u};//4*20

  Eigen::Map<Eigen::Matrix<float, kRefSize, kRefSize * kSamples>>
    acado_W_{acadoVariables.W};//16*320

  Eigen::Map<Eigen::Matrix<float, kEndRefSize, kEndRefSize>>
    acado_W_end_{acadoVariables.WN};//12*12

  Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
    acado_lower_bounds_{acadoVariables.lbValues};//4*20

  Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
    acado_upper_bounds_{acadoVariables.ubValues};//4*20

  Eigen::Matrix<float, kRefSize, kRefSize> W_ = (Eigen::Matrix<float, kRefSize, 1>() <<
    10 * Eigen::Matrix<float, 3, 1>::Ones(),//position
    10 * Eigen::Matrix<float, 3, 1>::Ones(),//attitude
    10 * Eigen::Matrix<float, 3, 1>::Ones(),//velocity
    10 * Eigen::Matrix<float, 3, 1>::Ones(),//angular velocity
    10 * Eigen::Matrix<float, 3, 1>::Ones(),
    10//thust
    ).finished().asDiagonal();//初始化权重矩阵，这里的权重数值可能得改

  Eigen::Matrix<float, kEndRefSize, kEndRefSize> WN_ =
    W_.block(0, 0, kEndRefSize, kEndRefSize);
  
  bool acado_is_prepared_{false};
  const float dt_{0.1};//this value should be read from mpc model, cannot be set randomly
};

} // namespace MPC