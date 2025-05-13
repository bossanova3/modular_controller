#ifndef MODULAR_CONTROLLER_H_
#define MODULAR_CONTROLLER_H_

#include <termios.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "tf/tf.h"
#include "geometry_msgs/Pose.h"

#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#define NUM_OF_JOINT 4
#define DELTA 0.02
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

class ModularController
{
 public:
  ModularController();
  ~ModularController();

  // update
  void printText();
  void setGoal(char ch);

 private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;
  double roll, pitch, yaw, gripper, rollL, pitchL, yawL, gripperL;
  double j1, j2, j3, j4, j1L, j2L, j3L, j4L;
  int servo, servoL;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  ros::Subscriber hand_pose_sub_;
  ros::Subscriber gripper_sub_;
  ros::Subscriber servos_sub_;
  ros::Subscriber hand_pose_sub_L_;
  ros::Subscriber gripper_sub_L_;
  ros::Subscriber servos_sub_L_;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void handPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void gripperCallback(const std_msgs::Bool::ConstPtr& msg);
  void servosCallback(const std_msgs::Int32::ConstPtr& msg);
  void handPoseLCallback(const geometry_msgs::Pose::ConstPtr& msg);
  void gripperLCallback(const std_msgs::Bool::ConstPtr& msg);
  void servosLCallback(const std_msgs::Int32::ConstPtr& msg);

  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  struct termios oldt_;

  void disableWaitingForEnter(void);
  void restoreTerminalSettings(void);
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
  double getJ2();
  double getJ3();
  double getJ4();
  double getRoll();
  double getPitch();
  double getYaw();
  double getGripper();
  int getServo();
};

#endif //MODULAR_CONTROLLER_H_
