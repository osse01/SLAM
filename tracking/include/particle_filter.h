#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <Eigen/Dense>
#include <algorithm>  // For std::fill
#include <random>     // For random number generators
#include <vector>     // For std::vector (if not in your header)
#include <functional>



/*
 * Particle filter function
    * This function implements a basic particle filter algorithm that updates particles based on measurements
 * and resamples them based on their weights. The motion model and sensor model are provided as function pointers.
 * @param initial_particle Initial particle state
 * @param measurements Vector of measurements to process
 * @param model Function to compute the model prediction for a particle given a measurement
 * @param num_particles Number of particles in the filter
 * @param num_iterations Number of iterations to run the filter
 * @param resample_threshold Threshold for resampling particles based on their weights
 * @param likelihood Function to compute the likelihood of a particle given a measurement

 * @return The likeliest particles after processing the measurements
*/
Eigen::VectorXd particle_filter(Eigen::VectorXd initial_particle,
                                const Eigen::VectorXd& measurement,
                                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)> motion_model,
                                const std::function<Eigen::VectorXd(const Eigen::VectorXd&)> sensor_model,
                                const Eigen::MatrixXd& cov_sys,
                                int num_particles = 1000,
                                int num_iterations = 10,
                                double resample_threshold = 0.5);

#endif // PARTICLE_FILTER_H