#include <Eigen/Core>
#include <functional>
#include <iostream>
#include <fstream>
#include <cmath>
#include "../include/armijo.h"
#include "../include/levenberg_marquardt.h"


// 2D Rosenbrock function
Eigen::VectorXf rosenbrock_residual(const Eigen::VectorXf& x) {
    Eigen::VectorXf r(2);
    r(0) = 10.0f * (x(1) - x(0)*x(0));  // f1(x) = 10(y - x²)
    r(1) = 1.0f - x(0);                  // f2(x) = 1 - x
    return r;
}
// Analytical Jacobian for 2D Rosenbrock
Eigen::MatrixXf rosenbrock_jacobian(const Eigen::VectorXf& x) {
    Eigen::MatrixXf J(2, 2);
    // df1/dx = -20x, df1/dy = 10
    J(0, 0) = -20.0f * x(0); 
    J(0, 1) = 10.0f;
    // df2/dx = -1, df2/dy = 0
    J(1, 0) = -1.0f;
    J(1, 1) = 0.0f;
    return J;
}

// 4D Powell's function
Eigen::VectorXf powell_residual(const Eigen::VectorXf& x) {
    Eigen::VectorXf r(4);
    r(0) = x(0) + 10.0f * x(1);
    r(1) = std::sqrt(5.0f) * (x(2) - x(3));
    r(2) = (x(1) - 2.0f * x(2)) * (x(1) - 2.0f * x(2));
    r(3) = std::sqrt(10.0f) * (x(0) - x(3)) * (x(0) - x(3));
    return r;
}

// Analytical Jacobian for 4D Powell's function
Eigen::MatrixXf powell_jacobian(const Eigen::VectorXf& x) {
    Eigen::MatrixXf J(4, 4);
    
    // Row 1: df1/dx = [1, 10, 0, 0]
    J(0, 0) = 1.0f;    J(0, 1) = 10.0f; 
    J(0, 2) = 0.0f;    J(0, 3) = 0.0f;
    
    // Row 2: df2/dx = [0, 0, √5, -√5]
    J(1, 0) = 0.0f;    J(1, 1) = 0.0f;
    J(1, 2) = std::sqrt(5.0f);  
    J(1, 3) = -std::sqrt(5.0f);
    
    // Row 3: df3/dx = [0, 2(y-2z), -4(y-2z), 0]
    float term = x(1) - 2.0f * x(2);
    J(2, 0) = 0.0f;    
    J(2, 1) = 2.0f * term;
    J(2, 2) = -4.0f * term;
    J(2, 3) = 0.0f;
    
    // Row 4: df4/dx = [2√10(x-z), 0, 0, -2√10(x-z)]
    term = std::sqrt(10.0f) * (x(0) - x(3));
    J(3, 0) = 2.0f * term;  
    J(3, 1) = 0.0f;
    J(3, 2) = 0.0f;    
    J(3, 3) = -2.0f * term;
    
    return J;
}

int main() {
    

    // Eigen::VectorXf rosenbrock_x(2);
    // rosenbrock_x << -1.5f, 2.0f;

    // levenberg_marquardt(rosenbrock_x, rosenbrock_residual, rosenbrock_jacobian);

    Eigen::VectorXf powell_x(4);
    powell_x << -2.5f, 3.7f, -4.1f, 1.8f;
    levenberg_marquardt(powell_x, powell_residual, powell_jacobian);

    return 0;
}