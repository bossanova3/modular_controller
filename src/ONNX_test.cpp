#include <iostream>
#include <onnxruntime_cxx_api.h>

int main() {
    try {
        // Ruta al modelo ONNX
        std::string model_path = "modelo_rnn.onnx";

        // Inicializar ONNX Runtime
        std::cout << "Inicializando ONNX Runtime..." << std::endl;
        Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "ONNXRuntimeModel");

        // Crear opciones de sesión
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1); // Número de hilos (opcional)

        // Crear la sesión
        Ort::Session session(env, model_path.c_str(), session_options);

        std::cout << "Modelo cargado correctamente." << std::endl;

    } catch (const Ort::Exception& e) {
        std::cerr << "Error al cargar el modelo: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
