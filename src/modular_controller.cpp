#include "modular_controller/modular_controller.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <fstream>
#include <onnxruntime_cxx_api.h>

double jo1, j2, j3, j4;
float rnn_roll, rnn_pitch, rnn_yaw;
bool inicio;

Ort::Env* env;
Ort::Session* session;
// Nombres de input/output del modelo
const char* input_names[] = {"input"};
const char* output_names[] = {"dense"};

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
  jo1 = 0, j2 = 0, j3 = 0, j4 = 0;
  servo = 0;

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
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/robot2/goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/robot2/goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/robot2/goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/robot2/goal_tool_control");
}

void ModularController::initSubscriber()
{
  hand_pose_sub_ = node_handle_.subscribe("hand_pose", 1000, &ModularController::handPoseCallback, this);
  gripper_sub_ = node_handle_.subscribe("gripper_state", 1000, &ModularController::gripperCallback, this);
  servos_sub_ = node_handle_.subscribe("selected_joint", 1000, &ModularController::servosCallback, this);
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

void ModularController::handPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

      // -------- PREPARAR LA ENTRADA PARA LA RNN --------
      std::vector<float> input_tensor_values = {static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw)};
      std::array<int64_t, 3> input_shape = {1, 1, 3}; // batch_size=1, seq_len=1, input_size=3
  
      Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
      Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
          memory_info,
          input_tensor_values.data(),
          input_tensor_values.size(),
          input_shape.data(),
          input_shape.size()
      );
  
      // -------- HACER LA PREDICCIÓN CON LA RNN --------
      auto output_tensors = session->Run(
          Ort::RunOptions{nullptr},
          input_names,
          &input_tensor,
          1,
          output_names,
          1
      );
  
      // Obtener la salida
      float* output_data = output_tensors.front().GetTensorMutableData<float>();
  
      // Por ejemplo, si la RNN saca también 3 valores
      rnn_roll = output_data[0];
      rnn_pitch = output_data[1];
      rnn_yaw = output_data[2];
}

void ModularController::gripperCallback(const std_msgs::Bool::ConstPtr& msg){
  gripper = (msg->data) ? -0.01 : 0.01;
}

void ModularController::servosCallback(const std_msgs::Int32::ConstPtr& msg){
  servo = msg->data;
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
  return roll;
}

double ModularController::getPitch()
{
  return pitch;
}

double ModularController::getYaw()
{
  return yaw;
}

double ModularController::getJ2()
{
  return j2;
}

double ModularController::getJ3()
{
  return j3;
}

double ModularController::getJ4()
{
  return j4;
}

double ModularController::getGripper()
{
  return gripper;
}

int ModularController::getServo()
{
  return servo;
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
  printf("8 : vr teleop modular\n");
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
    printf("pitch = %.2f, roll = %.2f\n",pitch,roll);
    printf("rnn_pitch = %.2f, rnn_roll = %.2f\n",rnn_pitch,rnn_roll);

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    std::vector<double> joint_angle_gripper;
    double path_time = 0.035;
    jo1 = rnn_roll;

    if(inicio == true){
      joint_name.push_back("joint1"); joint_angle.push_back(kf1.update(jo1));
      joint_name.push_back("joint2"); joint_angle.push_back(kf2.update(j2));
      joint_name.push_back("joint3"); joint_angle.push_back(kf3.update(j3));
      joint_name.push_back("joint4"); joint_angle.push_back(kf4.update(j4));
      joint_angle_gripper.push_back(gripper);
      setToolControl(joint_angle_gripper);
      setJointSpacePath(joint_name, joint_angle, 0.9);
      sleep(1);
      inicio = false;
    }

    joint_name.push_back("joint1"); joint_angle.push_back(kf1.update(jo1));
    if(servo == 0)
    {
      j2 = rnn_pitch;
    }
    if(servo == 1)
    {
      j3 = rnn_pitch;
    }
    if(servo == 2)
    {
      j4 = rnn_pitch;
    }
    joint_name.push_back("joint2"); joint_angle.push_back(kf2.update(j2));
    joint_name.push_back("joint3"); joint_angle.push_back(kf3.update(j3));
    joint_name.push_back("joint4"); joint_angle.push_back(kf4.update(j4));
    joint_angle_gripper.push_back(gripper);
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
  ros::init(argc, argv, "modular_controller");

  try {
    env = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeModel");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);

    session = new Ort::Session(*env, "/home/scf-usuario2/catkin_ws/src/modular_controller/modelo_rnn.onnx", session_options);

    ROS_INFO("Modelo RNN cargado correctamente.");
  } catch (const Ort::Exception& e) {
    ROS_ERROR("Error cargando el modelo ONNX: %s", e.what());
    return -1;
  }

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
      jo1 = 0;
      j2 = 0;
      j3 = 0;
      j4 = 0;
    }
  }

  return 0;
}
