#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

struct OptimizationConfig {
    int nelx = 60;          // Elements in X direction
    int nely = 30;          // Elements in Y direction
    double volfrac = 0.40;  // Target volume fraction (40% mass retention)
    double penal = 3.0;     // SIMP penalty factor (p = 3)
    double rmin = 1.8;      // Filter radius (prevents checkerboard)
    int max_iter = 50;      // Maximum OC iterations
};

class TopologyOptimizer {
public:
    explicit TopologyOptimizer(const OptimizationConfig& config);
    void solve();
    void exportDensityMatrix(const std::string& filename) const;
    const std::vector<double>& getDensities() const { return x; }
    void setPassiveSolidRegion(int x_min, int x_max, int y_min, int y_max);
    double computeAverageDensity() const;

private:
    OptimizationConfig cfg;
    int nElements;
    int nNodes;
    std::vector<double> x;              // Element densities [0.001, 1.0]
    std::vector<double> dc;             // Objective sensitivities
    std::vector<bool> passive_solid;    // Non-design solid regions
    Eigen::MatrixXd KE;                 // 8x8 Element stiffness matrix

    void initializeElementStiffness();
    void filterSensitivities();
    void updateOptimalityCriteria();
};
