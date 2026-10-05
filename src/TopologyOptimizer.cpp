#include "TopologyOptimizer.hpp"
#include <iostream>
#include <fstream>
#include <cmath>

TopologyOptimizer::TopologyOptimizer(const OptimizationConfig& config)
    : cfg(config),
      nElements(config.nelx * config.nely),
      nNodes((config.nelx + 1) * (config.nely + 1)),
      x(nElements, config.volfrac),
      dc(nElements, 0.0) {
    initializeElementStiffness();
}

void TopologyOptimizer::initializeElementStiffness() {
    // Standard Plane Stress Quad (Q4) element stiffness matrix for nu = 0.3
    const double nu = 0.3;
    const double E = 1.0;
    KE.resize(8, 8);
    KE << 0.4945,  0.1786, -0.3022, -0.0137, -0.2473, -0.1786,  0.0549,  0.0137,
          0.1786,  0.4945,  0.0137,  0.0549, -0.1786, -0.2473, -0.0137, -0.3022,
         -0.3022,  0.0137,  0.4945, -0.1786,  0.0549, -0.0137, -0.2473,  0.1786,
         -0.0137,  0.0549, -0.1786,  0.4945,  0.0137, -0.3022,  0.1786, -0.2473,
         -0.2473, -0.1786,  0.0549,  0.0137,  0.4945,  0.1786, -0.3022, -0.0137,
         -0.1786, -0.2473, -0.0137, -0.3022,  0.1786,  0.4945,  0.0137,  0.0549,
          0.0549, -0.0137, -0.2473,  0.1786, -0.3022,  0.0137,  0.4945, -0.1786,
          0.0137, -0.3022,  0.1786, -0.2473, -0.0137,  0.0549, -0.1786,  0.4945;
    KE *= (E / (1.0 - nu * nu));
}

void TopologyOptimizer::filterSensitivities() {
    std::vector<double> dc_filtered = dc;
    for (int i = 0; i < cfg.nelx; ++i) {
        for (int j = 0; j < cfg.nely; ++j) {
            double sum = 0.0;
            double weight_sum = 0.0;
            for (int k = std::max(i - (int)cfg.rmin, 0); k <= std::min(i + (int)cfg.rmin, cfg.nelx - 1); ++k) {
                for (int l = std::max(j - (int)cfg.rmin, 0); l <= std::min(j + (int)cfg.rmin, cfg.nely - 1); ++l) {
                    double dist = std::sqrt((i - k) * (i - k) + (j - l) * (j - l));
                    double weight = std::max(0.0, cfg.rmin - dist);
                    int idx = k * cfg.nely + l;
                    sum += weight * x[idx] * dc[idx];
                    weight_sum += weight;
                }
            }
            int curr = i * cfg.nely + j;
            if (weight_sum > 0.0 && x[curr] > 1e-6) {
                dc_filtered[curr] = sum / (x[curr] * weight_sum);
            }
        }
    }
    dc = dc_filtered;
}

void TopologyOptimizer::updateOptimalityCriteria() {
    double l1 = 0.0;
    double l2 = 100000.0;
    const double move = 0.2;
    std::vector<double> xnew(nElements, 0.0);

    while ((l2 - l1) > 1e-4) {
        double lmid = 0.5 * (l2 + l1);
        for (int i = 0; i < nElements; ++i) {
            double step = x[i] * std::sqrt(-dc[i] / lmid);
            xnew[i] = std::max(0.001, std::max(x[i] - move, std::min(1.0, std::min(x[i] + move, step))));
        }
        double current_vol = 0.0;
        for (double v : xnew) current_vol += v;
        if (current_vol - (cfg.volfrac * nElements) > 0) {
            l1 = lmid;
        } else {
            l2 = lmid;
        }
    }
    x = xnew;
}

void TopologyOptimizer::solve() {
    std::cout << "Starting SIMP Topology Optimization Iterations..." << std::endl;
    for (int iter = 1; iter <= cfg.max_iter; ++iter) {
        // Analytical cantilever beam strain energy proxy:
        double compliance = 0.0;
        for (int i = 0; i < cfg.nelx; ++i) {
            for (int j = 0; j < cfg.nely; ++j) {
                int e = i * cfg.nely + j;
                // Cantilever load moment distribution approximation
                double arm = (cfg.nelx - i);
                double strain_energy = (arm * arm * 0.1) / (1.0 + std::abs(j - cfg.nely / 2.0));
                
                // SIMP intermediate penalty: E(x) = E_min + x^p * (E0 - E_min)
                compliance += std::pow(x[e], cfg.penal) * strain_energy;
                dc[e] = -cfg.penal * std::pow(x[e], cfg.penal - 1.0) * strain_energy;
            }
        }

        filterSensitivities();
        updateOptimalityCriteria();

        if (iter % 10 == 0 || iter == cfg.max_iter) {
            std::cout << "Iter: " << iter << " | Compliance (Strain Energy): " << compliance << std::endl;
        }
    }
}

void TopologyOptimizer::exportDensityMatrix(const std::string& filename) const {
    std::ofstream out(filename);
    for (int j = 0; j < cfg.nely; ++j) {
        for (int i = 0; i < cfg.nelx; ++i) {
            out << x[i * cfg.nely + j] << (i == cfg.nelx - 1 ? "" : ",");
        }
        out << "\n";
    }
    std::cout << "Exported density field to: " << filename << std::endl;
}
