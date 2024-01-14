#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <tf/tf.h>

class ThirdPolynomial{
public:
  double pos;
  double vel;
private:
  double a;//3, t^3
  double b;//2, t^2
  double c;//1, t^1
  double d;//0, t^0
  bool coeff_set = false;

public:  
  void setCoeff(const double& current_val,const double& desired_val, const double& t){
  d = current_val;
  c = 0;
  b = 3*(desired_val - d) / (t*t);
  a = - 2 * b/3.0/t;
  coeff_set = true;
  }
  // void setCoeff(const Eigen::Vector3d& current_vector, const Eigen::Vector3d& desired_vec, const double& t){

  // }
  void calPos(const double& t){
    if(coeff_set){
      pos = a*t*t*t+ b *t*t + c *t + d;
    }
    else{
      pos =0;
      ROS_ERROR("Coefficient not set!");
    }
    // std::cout << a << " " << b << " " << c << " " << d << std::endl;
  }

  void calVel(const double& t){
    if(coeff_set){
      vel = 3*a*t*t + 2*b*t + c;
    }
    else{
      vel = 0;
      ROS_ERROR("Coefficient not set!");
    }
  }

  double getPos(){
    return pos;
  }
};


static const int64_t kNanoSecondsInSecond = 1000000000;//ns
static const int64_t kTest = 0.01;
static const double kDeltaT = 0.01;//s
static const float DEG_2_RAD = M_PI / 180.0;

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0), pitch(0.0),yaw(0.0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float _pitch, float _yaw)
      : position(x, y, z), pitch(_pitch), yaw(_yaw), waiting_time(t) {
  }

  Eigen::Vector3d position;
  double yaw;
  double pitch;
  double waiting_time;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "waypoint_publisher_3rd_polynomial");
  ros::NodeHandle nh;
  ros::Publisher wp_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  // ros::Subscriber sub = nh.subscribe("center_state", 10, &callback);
  ROS_INFO("Started waypoint_publisher_3rd_polynomial.");

  ros::V_string args;
  ros::removeROSArgs(argc, argv, args);

  double delay;
  // ROS_INFO("x: %f",std::stof(args.at(1)));
  // ROS_INFO("y: %f",std::stof(args.at(2)));
  // ROS_INFO("z: %f",std::stof(args.at(3)));
  // ROS_INFO("pitch: %f",std::stof(args.at(4)));
  // ROS_INFO("current_pitch: %f",std::stof(args.at(5)));
  // ROS_INFO("delay: %f",std::stof(args.at(6)));


  ROS_INFO("x: %f",std::stof(args.at(1)));
  ROS_INFO("current x: %f",std::stof(args.at(2)));

  ROS_INFO("y: %f",std::stof(args.at(3)));
  ROS_INFO("current y: %f",std::stof(args.at(4)));

  ROS_INFO("z: %f",std::stof(args.at(5)));
  ROS_INFO("current z: %f",std::stof(args.at(6)));

  ROS_INFO("pitch: %f",std::stof(args.at(7)));
  ROS_INFO("current pitch: %f",std::stof(args.at(8)));

  ROS_INFO("yaw: %f",std::stof(args.at(9)));
  ROS_INFO("current_yaw: %f",std::stof(args.at(10)));

  ROS_INFO("delay: %f",std::stof(args.at(11)));


  if (args.size() == 11) {
    delay = 1.0;
  } else if (args.size() == 12) {
    delay = std::stof(args.at(11));
  } else {
    ROS_ERROR("Usage: waypoint_publisher <x_d> <x_sen> <y_d> <y_sen> <z_d> <z_sen> <pitch_d> <pitch_sen> <yaw_d> <yaw_sen> [<delay>]\n");
    return -1;
  }
//-------------------------------------------------------------------------------------
  Eigen::Vector3d desired_position(std::stof(args.at(1)), std::stof(args.at(3)),
                                   std::stof(args.at(5)));

  Eigen::Vector3d current_position(std::stof(args.at(2)), std::stof(args.at(4)),
                                   std::stof(args.at(6)));

  double desired_pitch = std::stof(args.at(7)) * DEG_2_RAD;
  double current_pitch = std::stof(args.at(8))*DEG_2_RAD;

  double desired_yaw = std::stof(args.at(9)) * DEG_2_RAD;
  double current_yaw = std::stof(args.at(10)) * DEG_2_RAD;
//-------------------------------------------------------------------------------------
  ROS_INFO("About to construct coefficient");
  ROS_INFO("Wait for 3 seconds");
  ros::Duration(3).sleep();

  ThirdPolynomial pitch_com;
  pitch_com.setCoeff(current_pitch,desired_pitch,delay);
  ThirdPolynomial yaw_com;
  yaw_com.setCoeff(current_yaw,desired_yaw,delay);
  ThirdPolynomial position_x_com;
  position_x_com.setCoeff(current_position.x(),desired_position.x(),delay);
  ThirdPolynomial position_y_com;
  position_y_com.setCoeff(current_position.y(),desired_position.y(),delay);
  ThirdPolynomial position_z_com;
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
    position_x_com.calPos(time);  position_x_com.calVel(time);
    position_y_com.calPos(time);  position_y_com.calVel(time);
    position_z_com.calPos(time);  position_z_com.calVel(time);
    yaw_com.calPos(time);
    pitch_com.calPos(time);
    trajectory_point.position_W = Eigen::Vector3d(position_x_com.pos,position_y_com.pos,position_z_com.pos);
    trajectory_point.velocity_W = Eigen::Vector3d(position_x_com.vel,position_y_com.vel,position_z_com.vel);
    mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &msg->points[i]);
    q=tf::createQuaternionMsgFromRollPitchYaw(0,pitch_com.pos,yaw_com.pos);
    msg->points[i].transforms.at(0).rotation = q;
  }


  // tf::Quaternion quat;
  // tf::quaternionMsgToTF(q,quat);
  // tf::Matrix3x3 test_matrix;
  // test_matrix.setRotation(quat);
  // std::cout << test_matrix[0][0] << " " << test_matrix[0][1] << " " << test_matrix[0][2] << " " << std::endl;
  // std::cout << test_matrix[1][0] << " " << test_matrix[1][1] << " " << test_matrix[1][2] << " " << std::endl;
  // std::cout << test_matrix[2][0] << " " << test_matrix[2][1] << " " << test_matrix[2][2] << " " << std::endl;
  
  // std::cout << "q: " << q.w << " " << q.x << " " << q.y << " " <<q.z<<std::endl;
  // std::cout << "quat: " << quat.getW() << " " << quat.getX() << " " << quat.getY() << " " <<quat.getZ()<<std::endl;

  ros::Duration(1).sleep();//问题症结所在，不知道为什么会这样，这里会卡住
  //但是对面应该是收到了消息，不然怎么激活控制器呢？
  wp_pub.publish(msg);

  ros::Duration(3).sleep();

  ros::spinOnce();
  ros::shutdown();

  return 0;
}

