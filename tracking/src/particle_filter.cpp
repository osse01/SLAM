#include "../include/particle_filter.h"

#include <iostream>
#include <fstream>

Eigen::VectorXd particle_filter(Eigen::VectorXd initial_particle,
                                const Eigen::VectorXd& measurement,
                                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)> motion_model,
                                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)> sensor_model,
                                const Eigen::MatrixXd& cov_sys,
                                int num_particles,
                                int num_iterations,
                                double resample_threshold)
{
    // A vector to hold all the particles
    std::vector<Eigen::VectorXd> particles(num_particles, initial_particle);
    std::default_random_engine gen;
    std::vector<double> weights(num_particles, 1.0 / num_particles);

    Eigen::MatrixXd cov_inv = cov_sys.inverse();


    // Add noise to initial particles
    std::normal_distribution<> noise(0.0, 0.1);
    for (auto& particle : particles) {
        particle[0] += noise(gen);
        particle[1] += noise(gen);
        particle[2] += noise(gen) * 0.1;
        particle[3] += noise(gen) * 0.1;
    }

    for (int iter = 0; iter < num_iterations; ++iter) {
        double total_weight = 0;

        // Measurement update step
        for (size_t idx = 0; idx < particles.size(); ++idx) {
            auto particle = particles[idx];
            // Calculate the error between the measurement and the predicted sensor model
            Eigen::VectorXd prediction = sensor_model(particle);
            Eigen::VectorXd error = measurement - prediction;
            
            // Use Mahalanobis distance with actual covariance
            double mahal_dist = error.transpose() * cov_inv * error;
            weights[idx] = std::exp(-0.5 * mahal_dist);
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

        // Calculate effective sample size
        double effective_sample_size = 0;
        for (const auto& w : weights) {
            effective_sample_size += w * w;
        }
        effective_sample_size = 1.0 / effective_sample_size;

        // Resample if effective sample size is below threshold
        if (effective_sample_size < resample_threshold * num_particles) {
            std::discrete_distribution<> dist(weights.data(), weights.data() + weights.size());
            std::vector<Eigen::VectorXd> new_particles;
            for (size_t i = 0; i < num_particles; ++i) {
                new_particles.push_back(particles[dist(gen)]);
            }
            particles = std::move(new_particles);
            weights.assign(num_particles, 1.0 / num_particles);
        }

        // Motion update step with noise
        std::normal_distribution<> motion_noise(0.0, 2.0); 
        for (size_t idx = 0; idx < particles.size(); ++idx) {
            particles[idx] = motion_model(particles[idx]);
            // Add process noise
            particles[idx][0] += motion_noise(gen);
            particles[idx][1] += motion_noise(gen);
            particles[idx][2] += motion_noise(gen) * 0.1;
            particles[idx][3] += motion_noise(gen) * 0.1;
        }
    }

    // Final measurement update
    double total_weight = 0;
    for (size_t idx = 0; idx < particles.size(); ++idx) {
        Eigen::VectorXd prediction = sensor_model(particles[idx]);
        Eigen::VectorXd error = measurement - prediction;
        double mahal_dist = error.transpose() * cov_inv * error;
        weights[idx] = std::exp(-0.5 * mahal_dist);
        total_weight += weights[idx];
    }

    if (total_weight < 1e-10) {
        std::fill(weights.begin(), weights.end(), 1.0 / num_particles);
    } else {
        for (auto& w : weights) {
            w /= total_weight;
        }
    }

    // Calculate weighted average as best estimate
    Eigen::VectorXd best_particle = Eigen::VectorXd::Zero(initial_particle.size());
    for (size_t idx = 0; idx < particles.size(); ++idx) {
        best_particle += particles[idx] * weights[idx];
    }

    return best_particle;
}