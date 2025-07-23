# **SLAM from Scratch in C++**  
My implementation of Simultaneous Localization and Mapping (SLAM) using **Eigen** and **C++ STL only**.  

## **Current Progress**  
### **Levenberg-Marquardt Optimizer**  
Implemented with **Armijo line search** and dynamic damping.  
- **Location**: `optimization/LevenbergMarquardt`  
- **Tests**: Rosenbrock (2D) and Powell (4D) functions (see [optimization/README.md](optimization/README.md)).

### **Particle Filter (PF)**
A basic PF to estimate the pose of a state. [See the README in tracking](tracking/README.md)

### **Extended Kalman Filter**
A basic PF to estimate the pose of a state. [See the README in tracking](tracking/README.md)

### **Upcoming Modules**  
1. Combine the filters to a Marginalized Particle Filter (MPF)
2. Change the function to handle sparse matrices (For SLAM)
3. Implement SLAM

---

## **Installation**  
### **Eigen (Required)**  

```bash
sudo apt update && sudo apt install libeigen3-dev
```

---

## **Benchmarks (Planned)**  
My vision is that these implementation will match what is on the market.

---

## **Code Style**  
- **Zero dependencies** beyond Eigen and STL.  
- **Header-only** where possible (e.g., `include/slam/optimization.hpp`).  
- **Benchmark-driven**: Each module includes tests like Rosenbrock/Powell.  

---

## **Why?**  
-  **Learn**: No black-box solvers (e.g., Ceres, g2o).  
-  **Control**: Eigen’s SIMD + STL parallelism for speed.  
-  **Test**: Compare against classic papers (e.g., PTAM, ORB-SLAM).  

---

Aligned with your optimization README’s style: **minimalist, math-aware, and results-focused**. Let me know if you’d like to add:  
- A **convergence plot** for pose-graph optimization.  
- Detailed **Jacobian derivation** for SLAM factors.  
- **ROS integration** notes (if applicable).
