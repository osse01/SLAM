#include "../include/levenberg_marquardt.h"
#include <iostream>
#include <fstream>
#include <iomanip>


Eigen::VectorXf& levenberg_marquardt(
    Eigen::VectorXf& x,
    const std::function<Eigen::VectorXf(const Eigen::VectorXf&)>& residual,
    const std::function<Eigen::MatrixXf(const Eigen::VectorXf&)>& jacobian,
    float lambda,
    float tol,
    int max_iterations)
{
    std::ofstream xdatafile("xdata.dat");
    std::ofstream fdatafile("fdata.dat");
    xdatafile << x.transpose() << std::endl;
    fdatafile << residual(x).squaredNorm() << std::endl;

    int n = x.size();
    Eigen::VectorXf r = residual(x);
    for (int iter = 0; iter < max_iterations; ++iter) {
        //lambda *= lambda_factor; // Increase damping factor

        // Compute residuals and Jacobian
        Eigen::VectorXf r_new = residual(x);
        Eigen::MatrixXf J = jacobian(x);
        Eigen::MatrixXf H = J.transpose() * J;

        if (!H.allFinite() || !r_new.allFinite() || !J.allFinite()) {
            std::cerr << "NaN or Inf detected in matrices!" << std::endl;
            exit(1);
        }

        if (r_new.squaredNorm() < r.squaredNorm()) {
            lambda *= 0.5f;
        } else {
            lambda *= 5.0f;
        }
        r = r_new;

        // Update Hessian with damping
        H += lambda * Eigen::MatrixXf::Identity(n, n);


        Eigen::VectorXf delta_x = H.ldlt().solve(-J.transpose() * r);

        auto cost = [&residual](const Eigen::VectorXf& x) {
            float cost = residual(x).squaredNorm();
            return cost;
        };
        auto grad_cost = [&jacobian, &residual](const Eigen::VectorXf& x) {
            Eigen::VectorXf grad = jacobian(x).transpose() * residual(x);
            return grad;
        };
        float alpha_init = std::min(1.0f, r.norm()/(J*delta_x).norm());
        float alpha = armijo(x, delta_x, alpha_init, 0.5, 0.5f, cost, grad_cost);
        x += alpha*delta_x;
        
        xdatafile << x.transpose() << std::endl;
        fdatafile << residual(x).squaredNorm() << std::endl;

        std::ostringstream iter_str;
        iter_str << "Iter " << iter;
        std::cout << std::left << std::setw(10) << iter_str.str()
        << ": x = ("
        << std::fixed << std::setprecision(6);
        for (int i = 0; i < x.size(); ++i) {
            std::cout << std::internal << std::setw(9) << x[i];  // Fixed width per element
            if (i != x.size() - 1) std::cout << ", ";  // Space between elements
        }
        std::cout << "), cost = "
        << std::scientific << std::setprecision(6) << std::noshowpos  // Hide + for cost
        << residual(x).squaredNorm()
        << std::endl;

        if (residual(x).squaredNorm() < tol) {
            std::cout << "Convergence achieved." << std::endl;
            break; // Convergence achieved
        }

    }
    xdatafile.close();
    fdatafile.close();
    system("gnuplot -p plot.plt");
    return x;
}
