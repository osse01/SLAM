#include "../include/particle_filter.h"
#include <iostream>

// Example sensor model function for the particle filter
const Eigen::VectorXd sensor_model(const Eigen::VectorXd& particle) {
    // Radar sensor model example
    // measurement is expected to be a 2D vector [range, angle]
    Eigen::VectorXd predicted(2);
    predicted[0] = std::sqrt(particle[0] * particle[0] + particle[1] * particle[1]); // Range
    predicted[1] = std::atan2(particle[1], particle[0]); // Angle
    return predicted;
}
// Example motion model function for the particle filter
const Eigen::VectorXd motion_model(const Eigen::VectorXd& particle) {
    // Simple motion model (Constant Velocity) that predicts the next state based on the current state and measurement
    Eigen::VectorXd predicted(4);
    std::default_random_engine gen;
    std::normal_distribution<> normal_dist(0, 1); // Normal distribution for noise

    predicted[0] = particle[0] + particle[2] + normal_dist(gen); // Update x1 with some noise
    predicted[1] = particle[1] + particle[3] + normal_dist(gen); // Update x2 with some noise
    predicted[2] = particle[2] + normal_dist(gen); // Update v1 with some noise
    predicted[3] = particle[3] + normal_dist(gen); // Update v2 with some noise
    return predicted;
}
// Example usage of the particle filter
int main() {
    Eigen::VectorXd initial_particle(4);
    initial_particle << 0, 0, 1, 1; // Initial state [x1, x2, v1, v2]

    std::vector<Eigen::VectorXd> measurements = {
        (Eigen::VectorXd(2) << 1.0, 0.5).finished(),
        (Eigen::VectorXd(2) << 2.0, 1.0).finished(),
        (Eigen::VectorXd(2) << 3.0, 1.5).finished()
    }; // Example measurements

    auto best_particles = particle_filter(initial_particle, measurements, motion_model, sensor_model);
    // Output the best particle
    for (size_t i = 0; i < best_particles.size(); ++i) {
        std::cout << "Best Particle at step " << i 
                  << ": x1 = " << best_particles[i][0]
                  << ", x2 = " << best_particles[i][1]
                  << ", v1 = " << best_particles[i][2]
                  << ", v2 = " << best_particles[i][3] << std::endl;
    }
    system("gnuplot -p plot.plt"); // Plot the results using gnuplot

    return 0;
}