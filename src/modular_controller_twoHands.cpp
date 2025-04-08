#include "modular_controller/modular_controller.h"
#include "ros/ros.h"
#include "tf/tf.h"

double jo1L, j2L, j3L, j4L;
bool inicio;

class KalmanFilter {
  private:
      double x_est; // Estado estimado
      double p_est; // Covarianza del error estimado
      double q;     // Ruido del proceso
      double r;     // Ruido de la medición
  public:
      KalmanFilter(double process_noise, double measurement_noise, double initial_estimate)
          : x_est(initial_estimate), p_est(1.0), q(process_noise), r(measurement_noise) {}
  
      double update(double measurement) {
          // Predicción
          double x_pred = x_est;
          double p_pred = p_est + q;
          
          // Corrección
          double k = p_pred / (p_pred + r); // Ganancia de Kalman
          x_est = x_pred + k * (measurement - x_pred);
          p_est = (1 - k) * p_pred;
          
          return x_est;
      }
  };

KalmanFilter kf1(0.001, 0.1, 0.0);
KalmanFilter kf2(0.001, 0.1, 0.0);
KalmanFilter kf3(0.01, 0.1, 0.0);
KalmanFilter kf4(0.01, 0.1, 0.0);

ModularController::ModularController()
: node_handle_(""),
  priv_node_handle_("~")
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);
  jo1L = 0, j2L = 0, j3L = 0, j4L = 0;
  servoL = 0;

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  disableWaitingForEnter();
  ROS_INFO("Ha comenzado el controlador modular");
}

ModularController::~ModularController()
{
  restoreTerminalSettings();
  ROS_INFO("Terminate OpenManipulator Joystick");
  ros::shutdown();
}

void ModularController::initClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/robot1/goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/robot1/goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/robot1/goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/robot1/goal_tool_control");
}

void ModularController::initSubscriber()
{
  hand_pose_sub_L_ = node_handle_.subscribe("hand_pose_left", 1000, &ModularController::handPoseLCallback, this);
  gripper_sub_L_ = node_handle_.subscribe("gripper_state_left", 1000, &ModularController::gripperLCallback, this);
  servos_sub_L_ = node_handle_.subscribe("selected_joint_left", 1000, &ModularController::servosLCallback, this);
}

void ModularController::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void ModularController::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void ModularController::handPoseLCallback(const geometry_msgs::Pose::ConstPtr& msg){
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Matrix3x3(q).getRPY(rollL, pitchL, yawL);
}

void ModularController::gripperLCallback(const std_msgs::Bool::ConstPtr& msg){
  gripperL = (msg->data) ? -0.01 : 0.01;
}

void ModularController::servosLCallback(const std_msgs::Int32::ConstPtr& msg){
  servoL = msg->data;
}

std::vector<double> ModularController::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> ModularController::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

double ModularController::getRoll()
{
  return rollL;
}

double ModularController::getPitch()
{
  return pitchL;
}

double ModularController::getYaw()
{
  return yawL;
}

double ModularController::getJ2()
{
  return j2L;
}

double ModularController::getJ3()
{
  return j3L;
}

double ModularController::getJ4()
{
  return j4L;
}

double ModularController::getGripper()
{
  return gripperL;
}

int ModularController::getServo()
{
  return servoL;
}

bool ModularController::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool ModularController::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool ModularController::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool ModularController::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if (goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void ModularController::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Controla Tu OpenManipulator!\n");
  printf("---------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("y : increase joint 1 angle\n");
  printf("h : decrease joint 1 angle\n");
  printf("u : increase joint 2 angle\n");
  printf("j : decrease joint 2 angle\n");
  printf("i : increase joint 3 angle\n");
  printf("k : decrease joint 3 angle\n");
  printf("o : increase joint 4 angle\n");
  printf("l : decrease joint 4 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("0 : pose modular test\n");
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("3 : pose modular parte 1\n");
  printf("4 : pose modular parte 2\n");
  printf("5 : pose modular parte 3\n");
  printf("6 : pose modular parte 4\n");
  printf("7 : algoritmo modular\n");
  printf("8 : vr teleop modular izquierda\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");

}

void ModularController::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  if (ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if (ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = -JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = -JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = -JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if (ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if (ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }
  else if (ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '0')
  {
    printf("input : 0 \tpose modular test\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '3')
  {
    printf("input : 3 \tpose modular parte 1\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.890);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.089);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.146);
    joint_name.push_back("joint4"); joint_angle.push_back(0.290);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '4')
  {
    printf("input : 4 \tpose modular parte 2\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(-0.890);
    joint_name.push_back("joint2"); joint_angle.push_back(0.373);
    joint_name.push_back("joint3"); joint_angle.push_back(0.675);
    joint_name.push_back("joint4"); joint_angle.push_back(-1.031);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '5')
  {
    printf("input : 5 \tpose modular parte 3\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.877);
    joint_name.push_back("joint2"); joint_angle.push_back(0.126);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.233);
    joint_name.push_back("joint4"); joint_angle.push_back(0.160);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '6')
  {
    printf("input : 6 \tpose modular parte 4\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 1.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.874);
    joint_name.push_back("joint2"); joint_angle.push_back(0.471);
    joint_name.push_back("joint3"); joint_angle.push_back(0.371);
    joint_name.push_back("joint4"); joint_angle.push_back(-0.870);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if (ch == '8')
  {
    printf("input : 8 \tpose modular VR\n");
    printf("pitch = %.2f, roll = %.2f",pitchL,rollL);

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    std::vector<double> joint_angle_gripper;
    double path_time = 0.035;
    jo1L = -rollL;

    if(inicio == true){
      joint_name.push_back("joint1"); joint_angle.push_back(kf2.update(jo1L));
      joint_name.push_back("joint2"); joint_angle.push_back(kf2.update(j2L));
      joint_name.push_back("joint3"); joint_angle.push_back(kf3.update(j3L));
      joint_name.push_back("joint4"); joint_angle.push_back(kf4.update(j4L));
      joint_angle_gripper.push_back(gripperL);
      setToolControl(joint_angle_gripper);
      setJointSpacePath(joint_name, joint_angle, 0.9);
      sleep(1);
      inicio = false;
    }

    joint_name.push_back("joint1"); joint_angle.push_back(kf1.update(jo1L));
    if(servoL == 0)
    {
      j2L = -pitchL;
    }
    if(servoL == 1)
    {
      j3L = -pitchL;
    }
    if(servoL == 2)
    {
      j4L = -pitchL;
    }
    joint_name.push_back("joint2"); joint_angle.push_back(kf2.update(j2L));
    joint_name.push_back("joint3"); joint_angle.push_back(kf3.update(j3L));
    joint_name.push_back("joint4"); joint_angle.push_back(kf4.update(j4L));
    joint_angle_gripper.push_back(gripperL);
    setToolControl(joint_angle_gripper);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

void ModularController::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void ModularController::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "modular_controller_left");

  ModularController modularController;

  char ch, ch2;
  int tiempoLimite;
  modularController.printText();
  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    modularController.printText();
    tiempoLimite = 0;
    inicio = true;
    ros::spinOnce();
    modularController.setGoal(ch);
    if(ch == '7'){
      ros::spinOnce();
      modularController.setGoal('g');
      sleep(3);
      ros::spinOnce();
      modularController.setGoal('1');
      sleep(3);
      ros::spinOnce();
      modularController.setGoal('3');
      sleep(2);
      ros::spinOnce();
      modularController.setGoal('4');
      sleep(3);
      ros::spinOnce();
      modularController.setGoal('f');
      sleep(2);
      ros::spinOnce();
      modularController.setGoal('3');
      sleep(1);
      ros::spinOnce();
      modularController.setGoal('5');
      sleep(3);
      ros::spinOnce();
      modularController.setGoal('6');
      sleep(2);
      ros::spinOnce();
      modularController.setGoal('g');
      sleep(2);
      ros::spinOnce();
      modularController.setGoal('5');
      sleep(1);
      ros::spinOnce();
      modularController.setGoal('2');
      sleep(3);
    }
    while(ch == '8' && tiempoLimite < 375){
      ros::spinOnce();
      usleep(40000);
      modularController.setGoal('8');
      tiempoLimite++;
    }
    if(tiempoLimite >= 375){
      printf("\nSe acabo los 30 segundos de uso");
      jo1L = 0;
      j2L = 0;
      j3L = 0;
      j4L = 0;
    }
  }

  return 0;
}
