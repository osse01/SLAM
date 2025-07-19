#ifndef LEVENBERG_MARQUARDT_H
#define LEVENBERG_MARQUARDT_H

#include <Eigen/Dense>
#include <functional>
#include "armijo.h"
/*
 * Levenberg-Marquardt optimization algorithm using armijo line search.
 * This function minimizes a non-linear least squares problem by iteratively updating
 * the parameters based on the residuals and Jacobian of the model.
 *
    * @param x Initial guess for the parameters
    * @param residual Function that computes the residuals
    * @param jacobian Function that computes the Jacobian matrix
    * @param lambda_init Initial value for the damping parameter
    * @param lambda_factor Factor by which to increase the damping parameter
    * @param tol Tolerance for convergence
    * @param max_iterations Maximum number of iterations
    * @return Optimized parameters
*/
Eigen::VectorXf& levenberg_marquardt(
    Eigen::VectorXf& x,
    const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& residual,
    const std::function<Eigen::MatrixXf(const Eigen::VectorXf&)>& jacobian,
    float lambda = 1.0f,
    float tol = 1e-15f,
    int max_iterations = 1000
);

#endif // LEVENBERG_MARQUARDT_H