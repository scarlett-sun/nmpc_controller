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
  bool setLimits(float min_thrust, float max_thrust, float max_taux, float max_tauy, float max_tauz);
  

  bool initializeAcadoVariables();

  /*设置初始状态*/
  bool setInitialState(const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> est_state){
    // Check if estimated and reference quaternion live in sthe same hemisphere.
    // 以后再检查吧
    acado_initial_state_ = est_state.template cast<float>();
    // if(acado_initial_state_.segment(3,4).dot(
    //   Eigen::Vector4f(acado_reference_states_.block(3,0,4,1)))<(T)0.0)
    // {
    //   acado_initial_state_.segment(3,4) = -acado_initial_state_.segment(3,4);
    // }
  }

  /*设置参考位姿（一个轨迹点）*/
  bool setReferencePose(const Eigen::Ref<const Eigen::Matrix<float, kStateSize, 1>> state);
  
  /*设置参考轨迹（一系列轨迹点）*/
  bool setTrajectory(
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

  // Eigen::Map<Eigen::Matrix<float, kOdSize, kSamples+1, Eigen::ColMajor>>
  //   acado_online_data_{acadoVariables.od};//0,currently not needed

  Eigen::Map<Eigen::Matrix<float, kRefSize, kRefSize * kSamples>>
    acado_W_{acadoVariables.W};//16*320

  Eigen::Map<Eigen::Matrix<float, kEndRefSize, kEndRefSize>>
    acado_W_end_{acadoVariables.WN};//12*12

  Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
    acado_lower_bounds_{acadoVariables.lbValues};//4*20

  Eigen::Map<Eigen::Matrix<float, 4, kSamples, Eigen::ColMajor>>
    acado_upper_bounds_{acadoVariables.ubValues};//4*20

  //以上是mpc_wrapper中对应acadoVariables的数据成员
  //共享内存，任何一方的修改会影响另一方的数值

  Eigen::Matrix<float, kRefSize, kRefSize> W_ = (Eigen::Matrix<float, kRefSize, 1>() <<
    10 * Eigen::Matrix<float, 3, 1>::Ones(),
    100 * Eigen::Matrix<float, 4, 1>::Ones(),
    10 * Eigen::Matrix<float, 3, 1>::Ones(),
    Eigen::Matrix<float, 2, 1>::Zero(),
    1, 10, 10, 1).finished().asDiagonal();//初始化权重矩阵，这里的权重数值可能得改

  Eigen::Matrix<float, kEndRefSize, kEndRefSize> WN_ =
    W_.block(0, 0, kEndRefSize, kEndRefSize);
  
  //W_和WN_是给单个state用的，而acado_W_和acado_WN_是给N个state用的

  bool acado_is_prepared_{false};
  const float dt_{0.1};//this value should be read from mpc model, cannot be set randomly
};



} // namespace MPC