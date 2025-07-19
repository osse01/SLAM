#include "../include/particle_filter.h"

#include <iostream>
#include <fstream>

std::vector<Eigen::VectorXd> particle_filter(Eigen::VectorXd initial_particle,
                                const std::vector<Eigen::VectorXd>& measurements,
                                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)> motion_model,
                                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)> sensor_model,
                                int num_particles,
                                int num_iterations,
                                double resample_threshold)
{

    // A vector to hold all the particles
    std::vector<Eigen::VectorXd> particles(num_particles, initial_particle);
    std::vector<Eigen::VectorXd> best_particles(measurements.size());

    std::default_random_engine gen;
    std::ofstream log_file("particle_filter.dat");
    log_file << initial_particle[0] << " " << initial_particle[1] << std::endl;

    for (int iter = 0; iter < measurements.size(); ++iter) {

        Eigen::VectorXd best_particle = Eigen::VectorXd::Zero(initial_particle.size());

        std::vector<double> weights(num_particles, 1.0 / num_particles);
        double total_weight = 0;

        // Measurement update step
        for (size_t idx = 0; idx < particles.size(); ++idx) {
            auto particle = particles[idx];
            // Calculate the error between the measurement and the predicted sensor model
            double e = (measurements[iter] - sensor_model(particle)).squaredNorm();
            // Calculate the weight based on the error
            // Using a Gaussian likelihood model
            weights[idx] = std::exp(-0.5 * e);
            total_weight += weights[idx];
        }
        if (total_weight < 1e-10) {
            std::fill(weights.begin(), weights.end(), 1.0 / num_particles);
            total_weight = 1.0;
        }

        // Normalize weights
        for (auto& w : weights) {
            w /= total_weight;
        }

        // Estimating the best particle
        // std::cout << "Number of particles: " << particles.size() << std::endl;
        // std::cout << "Particle dim: " << particles[0].size() 
        //   << " | Measurement dim: " << measurements[iter].size()
        //   << " | Sensor dim: " << sensor_model(particles[0]).size() << std::endl;
        for (size_t idx = 0; idx < particles.size(); ++idx) {
            best_particle += particles[idx] * weights[idx];
        }
        // std::cout << "Best particle: " << best_particle.transpose() << std::endl;
        best_particles[iter] = best_particle;
        log_file << best_particle[0] << " " << best_particle[1] << std::endl;

        // Resampling particles based on weights
        std::discrete_distribution<> dist(weights.data(), weights.data() + weights.size());
        std::vector<Eigen::VectorXd> new_particles;
        for (size_t i = 0; i < num_particles; ++i) {
            new_particles.push_back(particles[dist(gen)]);
        }
        particles = std::move(new_particles);
        weights.assign(num_particles, 1.0 / num_particles);

        // Motion update step
        for (size_t idx = 0; idx < particles.size(); ++idx) {
            auto particle = particles[idx];
            particles[idx] = motion_model(particle);
        }
    }
    log_file.close();


    return best_particles;
}