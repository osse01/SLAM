#include "../include/extended_kalman_filter.h"
#include <iostream>


Eigen::MatrixXd finiteDiff(const Eigen::VectorXd& state, const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& model) {
    const Eigen::VectorXd fx = model(state);
    Eigen::MatrixXd J(fx.size(), state.size());
    const double eps = 1e-3;
    for (int i = 0; i < state.size(); ++i) {
        Eigen::VectorXd state_plus = state;
        state_plus(i) += eps;
        Eigen::VectorXd state_minus = state;
        state_minus(i) -= eps;
        J.col(i) = (model(state_plus) - model(state_minus)) / (2 * eps);
    }
    return J;
}


State extended_kalman_filter(
    State& initial_state,
    const Eigen::VectorXd& measurements,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& motion_model,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& sensor_model,
    const Eigen::MatrixXd& process_noise_covariance,
    const Eigen::MatrixXd& measurement_noise_covariance
) {

    Eigen::MatrixXd P = initial_state.covariance;
    Eigen::VectorXd x = initial_state.state;
    Eigen::MatrixXd H = finiteDiff(x, sensor_model);
    Eigen::MatrixXd F = finiteDiff(x, motion_model);

    Eigen::MatrixXd S = H * P * H.transpose() + measurement_noise_covariance;
    Eigen::MatrixXd K = P * H.transpose() * 
        S.llt().solve(Eigen::MatrixXd::Identity(S.rows(), S.cols())); // K = P * H^T * S^-1

    Eigen::VectorXd error = measurements - sensor_model(x);

    x += K * error;
    P -= K * H * P;

    x = motion_model(x);
    P = finiteDiff(x, motion_model) * P * finiteDiff(x, motion_model).transpose() + process_noise_covariance;
    return State{x, P};
}