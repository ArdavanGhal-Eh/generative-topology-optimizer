# 🏗️ Generative Design & SIMP Topology Optimization Engine (C++20 & Python)

A high-performance **Topology Optimization & Generative Structural Synthesis Engine** developed in **Modern C++20** using the **SIMP (Solid Isotropic Material with Penalization)** algorithm and **Optimality Criteria (OC)** updates, with an automated **Python visualization and CAD boundary reconstruction pipeline**.

---

## 🎯 Real-World Applications & Cross-Industry Impact

### ⚙️ Mechanical, Aerospace & Automotive Engineering
* **Aerospace Bracket Lightweighting:** Automated material removal in titanium and aluminum aircraft brackets to achieve maximum stiffness-to-weight ratio.
* **Additive Manufacturing (3D Printing Design):** Synthesizing bio-inspired organic internal lattice infills and load paths that cannot be machined conventionally.
* **Automotive Chassis Optimization:** Optimizing vehicle suspension control arms and shock towers under multi-load fatigue conditions.

### 🌐 Cross-Industry & Software Applications
* **Generative Architectural Design:** Algorithmic layout design of high-rise building trusses, stadium canopies, and bridge support columns.
* **Semiconductor & Chip Routing (VLSI):** Density-based heat-sink dissipation channel layout on microprocessors.
* **Neural Network Structural Pruning:** Transfer of SIMP penalty formulations to prune redundant network weights while maintaining predictive accuracy.
* **Procedural Level & Mesh Generation in Gaming:** Generating natural structural ruin models and sci-fi organic architecture for Unreal Engine & Unity.

---

## 📐 Mathematical Formulation

The optimization problem seeks to minimize structural compliance (maximize global stiffness) under a fixed volume constraint:

$$\min_{\mathbf{x}} c(\mathbf{x}) = \mathbf{U}^T \mathbf{K} \mathbf{U} = \sum_{e=1}^{N} (E_{min} + x_e^p (E_0 - E_{min})) \mathbf{u}_e^T \mathbf{k}_0 \mathbf{u}_e$$

$$\text{Subject to:} \quad \frac{V(\mathbf{x})}{V_0} = \frac{\sum_{e=1}^N x_e}{N} \le f$$
$$0 < x_{min} \le x_e \le 1, \quad e = 1, \dots, N$$

### 1. Sensitivity Analysis & Filtering
$$\frac{\partial c}{\partial x_e} = -p x_e^{p-1} \mathbf{u}_e^T \mathbf{k}_0 \mathbf{u}_e$$
A mesh-independency filter with convolution radius $r_{min}$ is applied to eliminate numerical checkerboard artifacts.

### 2. Optimality Criteria (OC) Density Update
$$x_e^{new} = \begin{cases} \max(x_{min}, x_e - m) & \text{if } x_e B_e^\eta \le \max(x_{min}, x_e - m) \\ \min(1, x_e + m) & \text{if } x_e B_e^\eta \ge \min(1, x_e + m) \\ x_e B_e^\eta & \text{otherwise} \end{cases}$$
where $B_e = -\frac{\partial c / \partial x_e}{\lambda \partial V / \partial x_e}$ and $\lambda$ is the Lagrange multiplier determined via bisection.

---

## 🚀 Build & Run

### 1. Build and Run C++ Optimization Engine:
```bash
mkdir build && cd build
cmake ..
cmake --build .
./topology_optimizer
```

### 2. Visualize & Export High-Res Structural Load Paths:
```bash
cd ../python_api
pip install -r requirements.txt
python optimizer_runner.py
```

---

## 🛠️ Architecture & Tech Stack
- **Core Engine:** Modern C++20, `Eigen3`
- **Algorithm:** Density-based SIMP + Optimality Criteria (OC)
- **CAD & Visualization:** Python 3.10+, NumPy, Matplotlib
- **Artifacts:** Density Matrix CSV, High-Res PNG Heatmap

---

## 👨‍💻 Author
**Ardavan Ghal-Eh**  
Mechanical Engineering Student, Sharif University of Technology  
*Focus: Generative Design, Computational Mechanics & Industrial Optimization*
