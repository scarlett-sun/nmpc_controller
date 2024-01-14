#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf/tf.h>

class FifthPolynomial{
public:
  double pos;
  double vel;
  double acc;
private:
  double a;//5, t^5
  double b;//4, t^4
  double c;//3, t^3
  double d;//2, t^2
  double e;//1, t^1
  double f;//0, t^0
  bool coeff_set = false;

public:
  /*current_val: initial value, desired_val:final value, t: time duration*/  
  void setCoeff(const double& current_val,const double& desired_val, const double& t){
    if(t==0){
      ROS_ERROR("Time duration cannot be 0!");
      return;
    }
    f = current_val;//guarantee initial position continuous
    e = 0;//guarantee initial velocity zero
    d = 0;//guarantee initial acceleration zero
    a = 6*(desired_val-f)/std::pow(t,5);
    b = -5*a*t/2;
    c = (-5*a*t*t-4*b*t)/3;
    coeff_set = true;
  }
  // void setCoeff(const Eigen::Vector3d& current_vector, const Eigen::Vector3d& desired_vec, const double& t){

  // }
  void calPos(const double& t){
    if(coeff_set){
      pos = a*std::pow(t,5)+ b*std::pow(t,4) + c*std::pow(t,3) + d*std::pow(t,2) + e*std::pow(t,1) + f*std::pow(t,0);
    }
    else{
      pos =0;
      ROS_ERROR("Coefficient not set!");
    }
    // std::cout << a << " " << b << " " << c << " " << d << std::endl;
  }

  void calVel(const double& t){
    if(coeff_set){
      vel = 5*a*std::pow(t,4)+ 4*b*std::pow(t,3) + 3*c*std::pow(t,2) + 2*d*std::pow(t,1) + e*std::pow(t,0);
    }
    else{
      vel = 0;
      ROS_ERROR("Coefficient not set!");
    }
  }

  void calAcc(const double& t){
    if(coeff_set){
      acc = 4*5*a*std::pow(t,3)+ 3*4*b*std::pow(t,2) + 2*3*c*std::pow(t,1) + 1*2*d*std::pow(t,0);
    }
  }

};


static const int64_t kNanoSecondsInSecond = 1000000000;//ns
static const int64_t kTest = 0.01;
static const double kDeltaT = 0.01;//s
static const float DEG_2_RAD = M_PI / 180.0;

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0),yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _yaw)
      : position(x, y, z), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double waiting_time;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher_5rd_polynomial");
  ros::NodeHandle nh;
  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  // ros::Subscriber sub = nh.subscribe("center_state", 10, &callback);
  ROS_INFO("Started waypoint_publisher_3rd_polynomial.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;

  ROS_INFO("x: %f",std::stof(args.at(1)));
  ROS_INFO("current x: %f",std::stof(args.at(2)));

  ROS_INFO("y: %f",std::stof(args.at(3)));
  ROS_INFO("current y: %f",std::stof(args.at(4)));

  ROS_INFO("z: %f",std::stof(args.at(5)));
  ROS_INFO("current z: %f",std::stof(args.at(6)));

  ROS_INFO("yaw: %f",std::stof(args.at(7)));
  ROS_INFO("current_yaw: %f",std::stof(args.at(8)));

  ROS_INFO("delay: %f",std::stof(args.at(9)));


  if (args.size() == 9) {
    delay = 1.0;
  } else if (args.size() == 10) {
    delay = std::stof(args.at(9));
  } else {
    ROS_ERROR("Usage: waypoint_publisher <x_d> <x_sen> <y_d> <y_sen> <z_d> <z_sen> <yaw_d> <yaw_sen> [<delay>]\n");
    return -1;
  }
//-------------------------------------------------------------------------------------
  Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(3)),
                                   std::stof(args.at(5)));

  Eigen::Vector3d current_position(std::stof(args.at(2)), std::stof(args.at(4)),
                                   std::stof(args.at(6)));

  double desired_yaw = std::stof(args.at(7)) * DEG_2_RAD;
  double current_yaw = std::stof(args.at(8)) * DEG_2_RAD;
//-------------------------------------------------------------------------------------
  ROS_INFO("About to construct coefficient");
  ROS_INFO("Wait for 3 seconds");
  ros::Duration(3).sleep();

  FifthPolynomial yaw_com;
  yaw_com.setCoeff(current_yaw,desired_yaw,delay);
  FifthPolynomial position_x_com;
  position_x_com.setCoeff(current_position.x(),desired_position.x(),delay);
  FifthPolynomial position_y_com;
  position_y_com.setCoeff(current_position.y(),desired_position.y(),delay);
  FifthPolynomial position_z_com;
  position_z_com.setCoeff(current_position.z(),desired_position.z(),delay);

  int points_number = delay * 100;
  ROS_INFO("Start publishing waypoints.");
//--------------------------------------------------------------------------
  trajectory_msgs::MultiDOFJointTrajectoryPtr msg(new trajectory_msgs::MultiDOFJointTrajectory);
  msg->header.stamp = ros::Time::now();
  msg->points.resize(points_number);
  msg->joint_names.push_back("base_link");
  int64_t time_from_start_ns = 0;
  geometry_msgs::Quaternion q;
  mav_msgs::EigenTrajectoryPoint trajectory_point;
  for (size_t i = 0; i < points_number; ++i) {
    trajectory_point.time_from_start_ns = time_from_start_ns;
    time_from_start_ns += static_cast<int64_t>(kDeltaT * kNanoSecondsInSecond);
    double time = double(time_from_start_ns) / kNanoSecondsInSecond;
    
    position_x_com.calPos(time);  position_x_com.calVel(time);  position_x_com.calAcc(time);
    position_y_com.calPos(time);  position_y_com.calVel(time);  position_y_com.calVel(time);
    position_z_com.calPos(time);  position_z_com.calVel(time);  position_z_com.calAcc(time);
    yaw_com.calPos(time);         yaw_com.calVel(time);         yaw_com.calAcc(time);

    trajectory_point.position_W = Eigen::Vector3d(position_x_com.pos,position_y_com.pos,position_z_com.pos);
    trajectory_point.velocity_W = Eigen::Vector3d(position_x_com.vel,position_y_com.vel,position_z_com.vel);
    trajectory_point.acceleration_W = Eigen::Vector3d(position_x_com.acc,position_y_com.acc,position_z_com.acc);
    trajectory_point.setFromYaw(yaw_com.pos);
    trajectory_point.setFromYawRate(yaw_com.vel);
    trajectory_point.setFromYawAcc(yaw_com.acc);
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
  }

  ros::Duration(1).sleep();
  wp_pub.publish(msg);

  ros::Duration(3).sleep();

  ros::spinOnce();
  ros::shutdown();

  return 0;
}