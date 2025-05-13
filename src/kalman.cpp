#include <iostream>
#include <vector>
#include <random>

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

int main() {
    std::vector<double> data(100);
    
    // Primera mitad con valores 0, segunda mitad con valores 0.57
    for (int i = 0; i < 50; ++i) {
        data[i] = 0.0;
    }
    for (int i = 50; i < 100; ++i) {
        data[i] = 0.57;
    }

    // Aplicar el filtro de Kalman
    KalmanFilter kf(0.01, 0.1, 0.0);
    std::vector<double> filtered_data;
    for (double val : data) {
        filtered_data.push_back(kf.update(val));
    }

    // Imprimir resultados
    std::cout << "Valores originales vs. Filtrados:\n";
    for (size_t i = 0; i < data.size(); ++i) {
        std::cout << "Original: " << data[i] << "\tFiltrado: " << filtered_data[i] << "\n";
    }
    return 0;
}
