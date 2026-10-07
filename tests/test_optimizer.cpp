#include <iostream>
#include <cassert>
#include <cmath>
#include "TopologyOptimizer.hpp"

void test_initialization() {
    OptimizationConfig cfg;
    cfg.nelx = 20;
    cfg.nely = 10;
    cfg.volfrac = 0.50;
    cfg.max_iter = 5;

    TopologyOptimizer opt(cfg);
    const auto& densities = opt.getDensities();
    assert(densities.size() == 200);

    for (double d : densities) {
        assert(std::abs(d - 0.50) < 1e-6);
    }
    std::cout << "✅ [PASS] test_initialization\n";
}

void test_density_bounds_and_convergence() {
    OptimizationConfig cfg;
    cfg.nelx = 20;
    cfg.nely = 10;
    cfg.volfrac = 0.40;
    cfg.max_iter = 15;

    TopologyOptimizer opt(cfg);
    opt.solve();

    const auto& densities = opt.getDensities();
    for (double d : densities) {
        assert(!std::isnan(d));
        assert(d >= 0.001 - 1e-6);
        assert(d <= 1.0 + 1e-6);
    }

    double avg_d = opt.computeAverageDensity();
    assert(std::abs(avg_d - cfg.volfrac) < 0.05);

    std::cout << "✅ [PASS] test_density_bounds_and_convergence | Final Avg Density: " << avg_d << "\n";
}

void test_passive_solid_region() {
    OptimizationConfig cfg;
    cfg.nelx = 20;
    cfg.nely = 10;
    cfg.volfrac = 0.40;
    cfg.max_iter = 10;

    TopologyOptimizer opt(cfg);
    // Pin corner region (0..2, 0..2) as non-design solid fixing point
    opt.setPassiveSolidRegion(0, 2, 0, 2);
    opt.solve();

    const auto& densities = opt.getDensities();
    for (int i = 0; i <= 2; ++i) {
        for (int j = 0; j <= 2; ++j) {
            int e = i * cfg.nely + j;
            assert(std::abs(densities[e] - 1.0) < 1e-3);
        }
    }
    std::cout << "✅ [PASS] test_passive_solid_region | Passive solid region preserved at 1.0\n";
}

int main() {
    std::cout << "=== Running Generative Topology Optimizer Unit Tests ===\n";
    test_initialization();
    test_density_bounds_and_convergence();
    test_passive_solid_region();
    std::cout << "All topology optimization tests passed successfully!\n";
    return 0;
}
