/*#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose.h"
#include <fstream>


std::ofstream file("/home/scf-usuario2/catkin_ws/src/modular_controller/hand_pose_data_RNN.csv", std::ios::app);
/*class KalmanFilter {
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
KalmanFilter kf3(0.001, 0.1, 0.0); */
/*
void handPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    double roll, pitch, yaw;  //kfRoll, kfPitch, kfYaw;
    ROS_INFO("Hand position is \nx=%.2f, y=%.2f, z=%.2f", msg->position.x, msg->position.y, msg->position.z);
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    ROS_INFO("Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
/*   kfRoll = kf1.update(roll);
    kfPitch = kf2.update(pitch);
    kfYaw = kf3.update(yaw);
    ROS_INFO("Filtered Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", kfRoll, kfPitch, kfYaw); */
  /*  ROS_INFO("Filtered Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);
    // // Escribir datos en formato CSV
    if (file.is_open()) {
         file << roll << "," 
              << pitch << "," 
              << yaw << "\n";
     }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "hand_pose_test");
    ros::NodeHandle nh;
    // // Encabezado del CSV si es la primera vez que se ejecuta
     if (file.is_open()) {
         file << "roll,pitch,yaw,label\n";
     }

    ros::Subscriber topic_sub = nh.subscribe("hand_pose", 1000, handPoseCallback);

    ros::spin();

    return 0;
}*/

#include "ros/ros.h"
#include "tf/tf.h"
#include "geometry_msgs/Pose.h"
#include <fstream>
#include <onnxruntime_cxx_api.h>  // <<< AÑADIR ESTO

// Archivo CSV
std::ofstream file("/home/scf-usuario2/catkin_ws/src/modular_controller/hand_pose_data_RNN.csv", std::ios::app);

// Crear el entorno y la sesión como punteros globales
Ort::Env* env;
Ort::Session* session;


// Nombres de input/output del modelo
const char* input_names[] = {"input"};
const char* output_names[] = {"dense"};

void handPoseCallback(const geometry_msgs::Pose::ConstPtr& msg){
    double roll, pitch, yaw;
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    ROS_INFO("Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch, yaw);

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
    float rnn_roll = output_data[0];
    float rnn_pitch = output_data[1];
    float rnn_yaw = output_data[2];

    ROS_INFO("Predicted Hand orientation is \nroll=%.2f, pitch=%.2f, yaw=%.2f", rnn_roll, rnn_pitch, rnn_yaw);

    // Guardar en el CSV la salida de la red
    if (file.is_open()) {
         file << rnn_roll << "," << rnn_pitch << "," << rnn_yaw << "\n";
     }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "hand_pose_test");
    ros::NodeHandle nh;

    // Encabezado del CSV si es la primera vez que se ejecuta
    if (file.is_open() && file.tellp() == 0) {
        file << "roll,pitch,yaw\n";
    }

    // -------- INICIALIZAR ONNX RUNTIME Y CARGAR EL MODELO --------
    try {
        env = new Ort::Env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeModel");
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);

        session = new Ort::Session(*env, "/home/scf-usuario2/catkin_ws/src/modular_controller/modelo_rnn.onnx", session_options);

      /*  size_t num_outputs = session->GetOutputCount();
        Ort::AllocatorWithDefaultOptions allocator;
        for (size_t i = 0; i < num_outputs; ++i) {
            char* output_name_cstr = session->GetOutputNameAllocated(i, allocator).release(); // 
            ROS_INFO("Output %zu name: %s", i, output_name_cstr);
            allocator.Free(output_name_cstr);    
        }*/

        ROS_INFO("Modelo RNN cargado correctamente.");
    } catch (const Ort::Exception& e) {
        ROS_ERROR("Error cargando el modelo ONNX: %s", e.what());
        return -1;
    }

    // Suscribirse al topic
    ros::Subscriber topic_sub = nh.subscribe("hand_pose", 1000, handPoseCallback);

    ros::spin(); }