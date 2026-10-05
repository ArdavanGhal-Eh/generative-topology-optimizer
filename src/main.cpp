#include <iostream>
#include "TopologyOptimizer.hpp"

int main() {
    std::cout << "================================================================" << std::endl;
    std::cout << "🚀 Generative Design & SIMP Topology Optimization Engine (C++20)" << std::endl;
    std::cout << "   Solving 2D Cantilever Beam with Volume Fraction = 40%" << std::endl;
    std::cout << "================================================================" << std::endl;

    OptimizationConfig config;
    config.nelx = 60;
    config.nely = 30;
    config.volfrac = 0.40;
    config.penal = 3.0;
    config.rmin = 2.0;
    config.max_iter = 40;

    TopologyOptimizer opt(config);
    opt.solve();
    opt.exportDensityMatrix("topology_density_field.csv");

    std::cout << "\n✅ Optimization Completed: Structural load paths successfully synthesized." << std::endl;
    return 0;
}
