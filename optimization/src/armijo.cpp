#include "../include/armijo.h"
#include <iostream>

float armijo(
    Eigen::VectorXf &x,
    Eigen::VectorXf &direction,  // (typically -gradient)
    float alpha_init,
    float beta,
    float sigma,
    const std::function<float(const Eigen::VectorXf&)>& objective_function,
    const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& gradient)
{
    const float f_current = objective_function(x);
    const Eigen::VectorXf grad = gradient(x);
    const float slope = grad.dot(direction);  // Should be NEGATIVE for descent

    // Safeguard against invalid directions
    if (slope >= 0) {
        std::cerr << "Warning: Not a descent direction (slope = " << slope << ")\n";
        return 0.0f;
    }

    float alpha = alpha_init;
    for (int i = 0; i < 100; ++i) {  // Reduced max iterations
        Eigen::VectorXf x_new = x + alpha * direction;
        const float f_new = objective_function(x_new);

        // Armijo condition
        if (f_new <= f_current + sigma * alpha * slope) {
            return alpha;
        }
        alpha *= beta;

        // Early termination if step becomes negligible
        if (alpha < 1e-10f) break;
    }

    return alpha;  // Return best found (even if condition not met)
}