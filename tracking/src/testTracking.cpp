#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <random>
#include <fstream>
#include <cmath>

#include "../include/extended_kalman_filter.h"
#include "../include/particle_filter.h"


const std::vector<Eigen::Vector2d> get_sensor_positions() {
    std::vector<Eigen::Vector2d> sensor_positions;
    // Read sensor positions from a file
    std::ifstream sensor_file("../data/mic_locations.txt");
    if (!sensor_file.is_open()) {
        throw std::runtime_error("Error opening sensor position file.");
    }
    std::string line;
    Eigen::Vector2d pos;
    while (getline(sensor_file, line)) {
        std::stringstream ss(line);
        ss >> pos[0] >> pos[1];
        sensor_positions.push_back(pos);
    }
    sensor_file.close();
    return sensor_positions;
}
const static std::vector<Eigen::Vector2d> sensor_positions = get_sensor_positions();
static double dt = 0.5; // Time step in seconds, will be set later based on measurements

const std::vector<Eigen::VectorXd> loadDataFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    // Read data row by row (each row is one measurement from all sensors)
    std::vector<Eigen::VectorXd> result_data;
    std::string line;
    
    while (std::getline(file, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        std::string value_str;
        
        while (std::getline(ss, value_str, ',')) {
            try {
                row.push_back(std::stod(value_str));
            } catch (const std::invalid_argument& e) {
                std::cerr << "Warning: Could not parse value '" << value_str << "'. Skipping." << std::endl;
            }
        }
        
        // Convert row to Eigen vector and add to result
        if (!row.empty()) {
            Eigen::VectorXd measurement(row.size());
            for (size_t i = 0; i < row.size(); ++i) {
                measurement[i] = row[i];
            }
            result_data.push_back(measurement);
        }
    }
    // Transpose the data
    std::vector<Eigen::VectorXd> transposed_data;
    int num_sensors = result_data.size();
    int num_measurements = result_data[0].size();

    transposed_data.resize(num_measurements);
    for (int i = 0; i < num_measurements; ++i) {
        transposed_data[i] = Eigen::VectorXd(num_sensors);
        for (int j = 0; j < num_sensors; ++j) {
            transposed_data[i][j] = result_data[j][i];
        }
    }
    result_data = transposed_data;
    file.close();

    return result_data;
}

/*
 * Measurement bias for each time step
*/
std::vector<Eigen::VectorXd> meas_bias(const std::vector<Eigen::VectorXd>& measurements) {
    std::vector<Eigen::VectorXd> bias;
    int size = measurements[0].size();
    for (const auto& data : measurements) {
        bias.push_back(data - Eigen::VectorXd::Constant(size, data.mean()));
    }
    return bias;
}

/*
 * Mean bias for stationary calibration
*/
Eigen::VectorXd mean_bias(const std::vector<Eigen::VectorXd>& measurements) {
    int size = measurements[0].size();
    Eigen::VectorXd sum = Eigen::VectorXd::Zero(size);
    
    // Calculate mean across all measurements for each sensor
    for (const auto& measurement : measurements) {
        sum += measurement;
    }
    return sum / measurements.size(); // This is the bias to subtract
}

Eigen::MatrixXd covariance(const std::vector<Eigen::VectorXd>& measurements) {
    int size = measurements[0].size();
    
    // For bias-corrected data, mean should be approximately zero
    Eigen::VectorXd overall_mean = Eigen::VectorXd::Zero(size);
    for (const auto& measurement : measurements) {
        overall_mean += measurement;
    }
    overall_mean /= measurements.size();
    
    // // Check that mean is close to zero after bias correction
    // std::cout << "Mean after bias correction: " << overall_mean.transpose() << std::endl;

    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(size, size);
    for (int i = 0; i < size; ++i) {
        for (int j = i; j < size; ++j) {
            double sum = 0;
            for (const auto& data : measurements) {
                sum += (data[i] - overall_mean[i]) * (data[j] - overall_mean[j]);
            }
            covariance(i,j) = sum / (double)(measurements.size() - 1);
            if (i!=j)
                covariance(j,i) = covariance(i,j);
        }
    }
    return covariance;
}

int NchooseK(int n, int k) {
        if (k > n) {
            return 0;
        }
        int result = 1;
        for (int i = 1; i <= k; ++i) {
            result = result * (n - k + i) / i;
        }
        return result;
    }

/*
 * Outlier detection based on range of sensor readings
 * Rejects measurements where max-min difference exceeds threshold
 */
std::vector<Eigen::VectorXd> outlier_detection(const std::vector<Eigen::VectorXd>& measurements) {
    std::vector<Eigen::VectorXd> filtered_measurements;
    
    if (measurements.empty()) return filtered_measurements;
    
    // Calculate statistics for Mahalanobis distance filtering
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(measurements[0].size());
    for (const auto& m : measurements) mean += m;
    mean /= measurements.size();
    
    Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(measurements[0].size(), measurements[0].size());
    for (const auto& m : measurements) {
        Eigen::VectorXd diff = m - mean;
        cov += diff * diff.transpose();
    }
    cov /= (measurements.size() - 1);
    
    for (const auto& measurement : measurements) {
        // Range-based filter
        double range = measurement.maxCoeff() - measurement.minCoeff();
        if (range > 5.0) continue; // Keep existing threshold
        
        // Mahalanobis distance filter
        Eigen::VectorXd diff = measurement - mean;
        double mahal_dist = std::sqrt(diff.transpose() * cov.inverse() * diff);
        if (mahal_dist > 3.0) continue; // 3-sigma threshold

        // Physical plausibility (TDOA should be reasonable for room size)
        bool physically_plausible = true;
        for (int i = 0; i < measurement.size(); ++i) {
            if (std::abs(measurement[i]) > 8.0) { // 8m max TDOA difference
                physically_plausible = false;
                break;
            }
        }
        
        if (physically_plausible) {
            filtered_measurements.push_back(measurement);
        }
    }
    
    return filtered_measurements;
}



// Update sensor model to return 3 TDOA distance differences
const Eigen::VectorXd sensor_model(const Eigen::VectorXd& particle) {

    Eigen::VectorXd distances = Eigen::VectorXd::Zero(4);
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d xi = sensor_positions[i];
        distances[i] = (particle.head<2>() - xi).norm(); // Keep as distances in meters
    }
    
    // Calculate distance differences relative to sensor 3
    Eigen::VectorXd tdoa_distances = Eigen::VectorXd::Zero(3);
    for (int i = 0; i < 3; ++i) {
        tdoa_distances[i] = distances[i] - distances[3]; // Distance differences in meters
    }
    return tdoa_distances;
}

const Eigen::VectorXd motion_model(const Eigen::VectorXd& particle) {
    // Simple motion model (Constant Velocity) that predicts the next state based on the current state and measurement
    Eigen::VectorXd predicted(4);
    double damping_factor = 1; // Damping factor to simulate friction or air resistance
    predicted.head<2>() = particle.head<2>() + dt * particle.segment<2>(2);
    predicted.tail<2>() = particle.tail<2>() * damping_factor; // Apply damping to velocity
    return predicted;
}


int main() {
    const double c = 343.0; // Speed of sound in air in m/s

    std::vector<Eigen::VectorXd> calibration_data = loadDataFromFile("../data/calibration.txt");
    
    // Mean bias calculation for calibration data 
    Eigen::VectorXd bias = mean_bias(calibration_data);
    
    // Remove bias from calibration data before calculating covariance
    for (auto& data : calibration_data) {
        data -= bias;
    }
    Eigen::MatrixXd cov = covariance(calibration_data);

    // Actual data
    std::vector<Eigen::VectorXd> measurements = loadDataFromFile("../data/tphat.txt");

    // Calculate mean time difference between consecutive measurements for dt
    double total_time_diff = 0.0;
    int time_diff_count = 0;
    for (int i = 1; i < measurements.size(); ++i) {
        // Use the first sensor's timestamp to calculate time difference
        double time_diff = measurements[i][0] - measurements[i-1][0];
        total_time_diff += time_diff;
        time_diff_count++;
    }
    dt = (time_diff_count > 0) ? total_time_diff / time_diff_count : 0.1;
    std::cout << "Calculated mean dt from timestamps: " << dt << " seconds" << std::endl;
    
    // Remove bias from measurements BEFORE converting to TDOA
    for (auto& data : measurements) {
        data -= bias;
    }
    // Convert measurements to TDOA format (3 distance differences relative to sensor 4)
    std::vector<Eigen::VectorXd> converted_measurements(measurements.size());
    for (int meas = 0; meas < measurements.size(); ++meas) {
        // Use sensor 4 (index 3) as reference, create 3 TDOA measurements
        converted_measurements[meas] = Eigen::VectorXd::Zero(3);
        for (int i = 0; i < 3; ++i) {
            converted_measurements[meas][i] = (measurements[meas][i] - measurements[meas][3]) * c; // Convert to distance differences
        }
    }

    // TDOA covariance for 3x3 matrix, scaled by c²
    Eigen::MatrixXd cov_sys = Eigen::MatrixXd::Zero(3,3);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            cov_sys(i,j) = (cov(i,j) - cov(i,3) - cov(3,j) + cov(3,3)) * c * c; // Scale by c² for distance units
        }
    }
    
    // Data statistics
    std::cout << "Reference TDOA Cov (3x3): \n" << cov_sys << std::endl;
    std::cout << "TDOA range in meters: min=" << converted_measurements[0].minCoeff() << ", max=" << converted_measurements[0].maxCoeff() << std::endl;

    std::cout << "Loaded " << converted_measurements.size() << " measurements." << std::endl;
    converted_measurements = outlier_detection(converted_measurements);
    std::cout << "Filtered to " << converted_measurements.size() << " measurements after outlier detection." << std::endl;
    
    Eigen::Matrix4d process_noise = Eigen::Matrix4d::Zero();
    process_noise(0,0) = 1e-2;  // Small position noise in x
    process_noise(1,1) = 1e-2;  // Small position noise in y  
    process_noise(2,2) = 1e-6;  // Velocity noise in x
    process_noise(3,3) = 1e-6;  // Velocity noise in y


    std::ofstream log_fileEKF("ekf_real.dat"); // For plotting
    std::ofstream log_filePF("particle_filter.dat");
    Eigen::Vector4d initial_state; 
    initial_state << 0, 0, 0, 0; // Initial state: [x, y, vx, vy]
    State stateEKF {initial_state, Eigen::Matrix4d::Identity()};
    Eigen::VectorXd statePF = initial_state;

    // Main loop for EKF
    std::cout << "Starting EKF with " << converted_measurements.size() << " measurements." << std::endl;
    for (int i = 0; i < converted_measurements.size(); ++i) {
        stateEKF = extended_kalman_filter(stateEKF, converted_measurements[i], motion_model, sensor_model, process_noise, cov_sys);
        statePF = particle_filter(statePF, converted_measurements[i], motion_model, sensor_model, cov_sys, 500, 5, 0.5);
        
        // Log the state for gnuplot
        log_fileEKF << stateEKF.state[0] << " " << stateEKF.state[1] << std::endl;
        log_filePF << statePF[0] << " " << statePF[1] << std::endl;
    }

    log_fileEKF.close();
    log_filePF.close();

    system("gnuplot ekf_real_plot.plt");
    system("gnuplot particle_filter_plot.plt");
    return 0;
}