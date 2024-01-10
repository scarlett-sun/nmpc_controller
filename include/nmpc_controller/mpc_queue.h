#ifndef NMPC_CONTROLLER_MPC_QUEUE_H_
#define NMPC_CONTROLLER_MPC_QUEUE_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <deque>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <visualization_msgs/Marker.h>

namespace mav_control {

typedef std::deque<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fDeque;

class MPCQueue
{
 public:
  MPCQueue(int mpc_queue_size);
  ~MPCQueue();

	void initializeQueue(const mav_msgs::EigenTrajectoryPoint& point,
			float controller_sampling_time, float prediction_sampling_time);

	void initializeQueue(const mav_msgs::EigenOdometry& odometry,
			float controller_sampling_time, float prediction_sampling_time);

	void initializeQueue(float controller_sampling_time,
			float prediction_sampling_time);

  void insertReference(const mav_msgs::EigenTrajectoryPoint& point);

  void insertReferenceTrajectory(const mav_msgs::EigenTrajectoryPointDeque& queue);

  void getQueue(Vector3fDeque& position_reference, Vector3fDeque& velocity_reference,
                        Vector3fDeque& acceleration_reference, std::deque<float>& yaw_reference,
                        std::deque<float>& yaw_rate_reference,std::deque<float>& yaw_acc_reference);

  void updateQueue();

  bool empty() const { return current_queue_size_ == 0; }
  int getCurrentQueueSize() const {return current_queue_size_;}

 private:
  int minimum_queue_size_;
  int mpc_queue_size_;
  const int maximum_queue_size_;
  int current_queue_size_;
  bool initialized_;

  float prediction_sampling_time_;
  float queue_dt_;

  //state reference
  Vector3fDeque position_reference_;//完整的插值后的，间隔为0.1s的trajectory
  Vector3fDeque velocity_reference_;
  Vector3fDeque acceleration_reference_;
  std::deque<float> yaw_reference_;
  std::deque<float> yaw_rate_reference_;
  std::deque<float> yaw_acc_reference_;

  float queue_start_time_;

  void clearQueue();
  void fillQueueWithPoint(const mav_msgs::EigenTrajectoryPoint& point);
  void pushBackPoint(const mav_msgs::EigenTrajectoryPoint& point);
  void popFrontPoint();
  void popBackPoint();
  void getLastPoint(mav_msgs::EigenTrajectoryPoint& point);
  void shrinkQueueToMinimum();

  //interpolate the reference queue to the controller update rate
  void linearInterpolateTrajectory(const mav_msgs::EigenTrajectoryPointDeque& input_queue,  mav_msgs::EigenTrajectoryPointDeque& output_queue);

  void eraseOriginalReference(const size_t& start_index);

  void printQueue();
};

}  // namespace mav_control

#endif  // NMPC_CONTROLLER_MPC_QUEUE_H_
