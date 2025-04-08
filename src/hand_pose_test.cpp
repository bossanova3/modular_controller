#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose.h"
#include <fstream>


// std::ofstream file("/home/scf-usuario2/catkin_ws/src/modular_controller/hand_pose_data.csv", std::ios::app);
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
KalmanFilter kf3(0.001, 0.1, 0.0);

void handPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    double roll, pitch, yaw, kfRoll, kfPitch, kfYaw;
    ROS_INFO("Hand position is \nx=%.2f, y=%.2f, z=%.2f", msg->position.x, msg->position.y, msg->position.z);
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
    kfRoll = kf1.update(roll);
    kfPitch = kf2.update(pitch);
    kfYaw = kf3.update(yaw);
    ROS_INFO("Filtered Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", kfRoll, kfPitch, kfYaw);

    // // Escribir datos en formato CSV
    // if (file.is_open()) {
    //     file << roll << "," 
    //          << pitch << "," 
    //          << yaw << "\n";
    // }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "hand_pose_test");
    ros::NodeHandle nh;
    // // Encabezado del CSV si es la primera vez que se ejecuta
    // if (file.is_open()) {
    //     file << "roll,pitch,yaw\n";
    // }

    ros::Subscriber topic_sub = nh.subscribe("hand_pose", 1000, handPoseCallback);

    ros::spin();

    return 0;
}