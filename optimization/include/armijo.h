#ifndef ARMIJO_H
#define ARMIJO_H

#include <Eigen/Dense>
#include <functional>

/*
 * Armijo line search algorithm.
    * This function finds a step size that satisfies the Armijo condition
    * for a given direction of descent.
    * @param x Current point in the optimization space
    * @param d Direction of descent
    * @param alpha Initial step size guess
    * @param beta Step size reduction factor
    * @param sigma Parameter for the Armijo condition
    * @param objective_function Function to evaluate the objective
    * @param gradient Function to compute the gradient at point x
    * @return Step size that satisfies the Armijo condition
*/
float armijo(
    Eigen::VectorXf& x,
    Eigen::VectorXf& d,
    float alpha_init,
    float beta,
    float sigma,
    const std::function<float(const Eigen::VectorXf&)>& f,
    const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& grad_f
);
#endif // ARMIJO_H