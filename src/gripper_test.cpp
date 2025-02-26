#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

ros::Publisher gripper_pub;

void gripperCallback(const std_msgs::Bool::ConstPtr& msg)
{
    std_msgs::Float64 gripper_msg;
    gripper_msg.data = msg->data ? 0.01 : -0.01;
    gripper_pub.publish(gripper_msg);

    ROS_INFO("Gripper %s", msg->data ? "close" : "open");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_test");
    ros::NodeHandle nh;

    gripper_pub = nh.advertise<std_msgs::Float64>("/gripper_action", 1000);

    ros::Subscriber topic_sub = nh.subscribe("/gripper_state", 1000, gripperCallback);

    ros::spin();
    return 0;
}