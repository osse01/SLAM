#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H
#include <Eigen/Dense>
#include <functional>

struct State {
    Eigen::VectorXd state;
    Eigen::MatrixXd covariance;
};

/*
 * Finite Difference scheme
 * This function computes the Jacobian matrix of the motion model at a given state.
 * It uses finite differences to approximate the derivatives.
 * @param state Current state vector
 * @param model Function to compute the model prediction for a particle given a measurement
 * @return The Jacobian matrix of the motion model at the given state
*/
Eigen::MatrixXd finiteDiff(const Eigen::VectorXd& state, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& model);


/*
 * Extended Kalman Filter function
 * This function implements an first-order Extended Kalman Filter (EKF1) algorithm that updates the state estimate
 * and covariance based on the motion model. It uses the Jacobian of the motion model
 * to linearize the system using first order jacobians around the current state estimate.
 * @param initial_state Initial state vector
 * @param initial_covariance Initial covariance matrix
 * @param motion_model Function to compute the model prediction for a particle given a measurement
 * @param process_noise_covariance Process noise covariance matrix
 * @param motion_jacobian Function to compute the Jacobian of the motion model at the current state
 * @return The updated state estimate and covariance matrix after processing the motion model
*/
State extended_kalman_filter(
    State& initial_state,
    const Eigen::VectorXd& measurements,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& motion_model,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& sensor_model,
    const Eigen::MatrixXd& process_noise_covariance,
    const Eigen::MatrixXd& measurement_noise_covariance
);


#endif // EXTENDED_KALMAN_FILTER_H