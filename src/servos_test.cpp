#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Int32.h"

int selected_joint = 0;

void jointCallback(const std_msgs::Int32::ConstPtr& msg){
    selected_joint = msg->data;
    ROS_INFO("Selected servo %d", selected_joint);
}
void handPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    double roll, pitch, yaw;
    ROS_INFO("Hand position is \nx=%.2f, y=%.2f, z=%.2f", msg->position.x, msg->position.y, msg->position.z);
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "hand_pose_test");
    ros::NodeHandle nh;

    ros::Subscriber topic_sub = nh.subscribe("hand_pose", 1000, handPoseCallback);
    ros::Subscriber joint_sub = nh.subscribe("selected_joint", 1000, jointCallback);

    ros::spin();

    return 0;
}