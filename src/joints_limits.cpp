#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// Limites
double joint_limits[4][2] = {
    {-3.142, 3.142},
    {-2.050, 1.571},
    {-1.571, 1.530},
    {-1.800, 2.000}
};

double validatePosition(int joint_index, double position){
    return std::max(std::min(position,
    joint_limits[joint_index][1]),
    joint_limits[joint_index][0]);
}

int main(int argc, char **argv){
    ros::init(argc, argv,"limites");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("/open_manipulator/goal_joint_space_path", 10);

    trajectory_msgs::JointTrajectory trajectory;
    trajectory_msgs::JointTrajectoryPoint point;

    

}