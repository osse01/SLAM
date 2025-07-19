# Levenberg-Marquardt
This is my implementation of Levenberg-Marquardt algorithm using Armijo line search with dynamic dampening.


To install Eigen:
```bash
sudo apt update
sudo apt install libeigen3-dev
```

## Benchmark tests
Rosenbrock and Powell function where tested to verify the implementation.

### Rosenbrock's Function in 2D
The Rosenbrock function is
$$
\begin{cases}
    r_0 (x) = 10(x_0 - x_1^2)\\
    r_1 (x) = 1 - x_1
\end{cases}
$$
Using start position $x = (-1.5, 2.0)^T$ we get
![Convergence Rosenbrock](images/benchmark_rosenbrock.png)
| Iter | x₀       | x₁       | Cost      |
|------|----------|----------|-----------|
| 1    | -1.376   |  1.883   | 5.653e+00 |
| ...  | ...      | ...      | ...       |
| 21   |  1.000   |  1.000   | 0.000e+00 |

### Powell's Function in 4D
The Powell function is
$$
\begin{cases} 
    r_0 = x_0 + 10x_1 \\ 
    r_1 = \sqrt{5}\;(x_2 - x_3) \\ 
    r_2 = (x_1 - 2x_2)^2 \\ 
    r_3 = \sqrt{10}(x_0 - x_3)^2 
\end{cases}
$$
![Convergence Rosenbrock](images/benchmark_powell.png)
|Iter| x₀      | x₁    | x₂      | x₃      | Cost     |
|----|---------|-------|---------|---------|----------|
|1   |-2.705   |0.454  |-2.746   |-0.554   |1.49e+03  |
|... |...      |...    |...      |...      |...       |
|26  |-0.000   |0.000  |-0.000   |-0.000   |9.03e-16  |
