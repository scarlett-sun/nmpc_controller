#include <memory>
#include <acado_optimal_control.hpp>
#include <acado_code_generation.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>

USING_NAMESPACE_ACADO
int main(int argc, char * const argv[ ])
{
  /*
  Switch between code generation and analysis.

  If CODE_GEN is true the system is compiled into an optimizaiton problem
  for real-time iteration and all code to run it online is generated.
  Constraints and reference structure is used but the values will be set on
  runtinme.

  If CODE_GEN is false, the system is compiled into a standalone optimization
  and solved on execution. The reference and constraints must be set in here.
  */
  const bool CODE_GEN = true;

  // System variables, x
  DifferentialState		p_x, p_y, p_z;//position, in world frame
  DifferentialState		phi, theta, psi;//attitude
  DifferentialState		v_x, v_y, v_z;//velocity, in world frame
  DifferentialState		w_x, w_y, w_z;//angular velocity, in body frame
//   DifferentialState		f1, f2, f3, f4;//the force produced by the individual propeller
//   Control				f1_dot, f2_dot, f3_dot, f4_dot;
  //先把基础的功能做出来再优化吧
  Control				tau_x, tau_y, tau_z, thrust;//in body frame
  DifferentialEquation  f;//state equation
  Function              h, hN;

  // Parameters with exemplary values. These are set/overwritten at runtime.
/*以下数值需要在整个功能包里统一*/
  const double t_start = 0.0;     // Initial time [s]
  const double t_end = 0.1;       // Time horizon [s]
  const double dt = 0.01;          // Discretization time [s]
  const int N = round(t_end/dt);  // Number of nodes

  const double g_z = 9.81;      // Gravity is everywhere [m/s^2]
  const double mass = 1.52;//运行时能否改变这些数据，例程写的是可以
  const double Jxx = 0.0347563;
  const double Jyy = 0.0458929;
  const double Jzz = 0.0977;
/*以上数值需要在整个功能包里统一*/

  //bounds are set online
  const double thrust_min = 0.1;         // Minimal thrust [N]
  const double thrust_max = 30;        // Maximal thrust [N]
  const double taux_max = 1;       // Maximal taux [Nm]
  const double tauy_max = 1;       // Maximal tauy [Nm]
  const double tauz_max = 1;       // Maximal tauz [Nm]

  f << dot(p_x) ==  v_x;
  f << dot(p_y) ==  v_y;
  f << dot(p_z) ==  v_z;

  f << dot(phi) ==   w_x * 1 + w_y * sin(phi)*tan(theta) + w_z * cos(phi)*tan(theta);
  f << dot(theta) == w_x * 0 + w_y * cos(phi)            + w_z * (-sin(phi));
  f << dot(psi) ==   w_x * 0 + w_y*sin(phi)/cos(theta)   + w_z * cos(phi)/cos(theta); 

  f << dot(v_x) == (cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*(thrust)/mass;
  f << dot(v_y) == (sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))*(thrust)/mass;
  f << dot(v_z) == (cos(theta)*cos(phi))*(thrust)/mass - g_z;

  f << dot(w_x) == (tau_x - (Jzz*w_z*w_y - Jyy*w_y*w_z))/Jxx;
  f << dot(w_y) == (tau_y - (Jxx*w_x*w_z - Jzz*w_z*w_x))/Jyy;
  f << dot(w_z) == (tau_z - (Jyy*w_x*w_y - Jxx*w_x*w_y))/Jzz;

  // 这里h是不是应该是误差啊，为啥输入的是state呢
  // Cost: Sum(i=0, ..., N-1){h_i' * Q * h_i} + h_N' * Q_N * h_N
  // Running cost vector consists of all states and inputs.
  h << p_x << p_y << p_z
	<< phi << theta << psi
	<< v_x << v_y << v_z
  << w_x << w_y << w_z
  << tau_x << tau_y << tau_z << thrust;
  // End cost vector consists of all states (no inputs at last state).

  hN << p_x << p_y << p_z
	<< phi << theta << psi
	<< v_x << v_y << v_z
	<< w_x << w_y << w_z;

  DMatrix Q(h.getDim(), h.getDim());
  Q.setIdentity();
  Q(0,0) = 100;   // p_x
  Q(1,1) = 100;   // p_y
  Q(2,2) = 100;   // p_z
  Q(3,3) = 10;   // phi
  Q(4,4) = 10;   // theta
  Q(5,5) = 10;   // psi
  Q(6,6) = 10;   // v_x
  Q(7,7) = 10;   // v_y
  Q(8,8) = 10;   // v_z
  Q(9,9) = 10;   // w_x
  Q(10,10) = 10;  // w_y
  Q(11,11) = 10;  // w_z
  Q(12,12) = 1;   // tau_x
  Q(13,13) = 1;   // tau_y
  Q(14,14) = 1;   // tau_z
  Q(15,15) = 1;   // thrust

  DMatrix QN(h.getDim(), h.getDim());
  QN.setIdentity();
  QN(0,0) = 100;   // p_x
  QN(1,1) = 100;   // p_y
  QN(2,2) = 100;   // p_z
  QN(3,3) = 10;   // phi
  QN(4,4) = 10;   // theta
  QN(5,5) = 10;   // psi
  QN(6,6) = 10;   // v_x
  QN(7,7) = 10;   // v_y
  QN(8,8) = 10;   // v_z
  QN(9,9) = 10;   // w_x
  QN(10,10) = 10;  // w_y
  QN(11,11) = 10;  // w_z

  //
  // Optimal Control Problem
  //
  OCP ocp( t_start, t_end, N );
  
  // For code generation, references are set during run time.
  BMatrix Q_sparse(h.getDim(), h.getDim());
  Q_sparse.setIdentity();
  BMatrix QN_sparse(hN.getDim(), hN.getDim());
  QN_sparse.setIdentity();
  ocp.minimizeLSQ( Q_sparse, h);
  ocp.minimizeLSQEndTerm( QN_sparse, hN );

  // Add system dynamics
  ocp.subjectTo( f );

  // Add constraints
  ocp.subjectTo(-taux_max <= tau_x <= taux_max);
  ocp.subjectTo(-tauy_max <= tau_y <= tauy_max);
  ocp.subjectTo(-tauz_max <= tau_z <= tauz_max);
  ocp.subjectTo(thrust_min <= thrust <= thrust_max);

  //
  // Export the code:
  //
  OCPexport mpc( ocp );

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  mpc.set(SPARSE_QP_SOLUTION,     FULL_CONDENSING_N2);  // due to qpOASES
  mpc.set(INTEGRATOR_TYPE, INT_IRK_GL4);
  mpc.set(NUM_INTEGRATOR_STEPS, N);
  mpc.set(QP_SOLVER, QP_QPOASES);
  mpc.set(HOTSTART_QP, YES);
  mpc.set(CG_USE_OPENMP,                    YES);       // paralellization
  mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,    NO);        // set on runtime
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);       // time-varying costs
  mpc.set( USE_SINGLE_PRECISION,        YES);           // Single precision
  
  // Do not generate tests, makes or matlab-related interfaces.
  mpc.set(GENERATE_TEST_FILE, NO);
  mpc.set(GENERATE_MAKE_FILE, NO);
  mpc.set(GENERATE_MATLAB_INTERFACE, NO);
  mpc.set( GENERATE_SIMULINK_INTERFACE, NO);

  // Finally, export everything.
  if (mpc.exportCode( "quadrotor_nmpc" ) != SUCCESSFUL_RETURN)
    exit( EXIT_FAILURE );
  mpc.printDimensionsQP( );
  
  return EXIT_SUCCESS;
}
