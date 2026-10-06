"""
3D Solid Mesh Extruder & STL Exporter
Part of SIMP Generative Topology Optimization Suite
Author: Ardavan Ghal-Eh | Sharif University of Technology
"""

import numpy as np


def export_topology_to_stl(density_matrix: np.ndarray, thickness_mm: float = 10.0, pixel_size_mm: float = 2.0, output_stl: str = "topology_optimized.stl"):
    """
    Extrudes a 2D SIMP density matrix into a watertight 3D solid STL mesh
    ready for 3D printer slicers (Cura/Prusa) or CAD import (SolidWorks).
    """
    nelx, nely = density_matrix.shape
    threshold = 0.5
    solid_mask = density_matrix >= threshold

    triangles = []

    def add_quad(p1, p2, p3, p4):
        # Two triangles per quad: (p1, p2, p3) and (p1, p3, p4)
        triangles.append((p1, p2, p3))
        triangles.append((p1, p3, p4))

    for x in range(nelx):
        for y in range(nely):
            if not solid_mask[x, y]:
                continue

            x0, x1 = x * pixel_size_mm, (x + 1) * pixel_size_mm
            y0, y1 = y * pixel_size_mm, (y + 1) * pixel_size_mm
            z0, z1 = 0.0, thickness_mm

            # Top face (z = z1)
            add_quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1])
            # Bottom face (z = z0)
            add_quad([x0, y1, z0], [x1, y1, z0], [x1, y0, z0], [x0, y0, z0])

            # Sides (if boundary)
            if x == 0 or not solid_mask[x - 1, y]:
                add_quad([x0, y0, z0], [x0, y1, z0], [x0, y1, z1], [x0, y0, z1])
            if x == nelx - 1 or not solid_mask[x + 1, y]:
                add_quad([x1, y0, z1], [x1, y1, z1], [x1, y1, z0], [x1, y0, z0])
            if y == 0 or not solid_mask[x, y - 1]:
                add_quad([x0, y0, z1], [x1, y0, z1], [x1, y0, z0], [x0, y0, z0])
            if y == nely - 1 or not solid_mask[x, y + 1]:
                add_quad([x0, y1, z0], [x1, y1, z0], [x1, y1, z1], [x0, y1, z1])

    # Write ASCII STL
    with open(output_stl, 'w') as f:
        f.write("solid topology_optimized_mesh\\n")
        for tri in triangles:
            # Approximate normal
            v1 = np.array(tri[1]) - np.array(tri[0])
            v2 = np.array(tri[2]) - np.array(tri[0])
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm > 1e-9:
                normal /= norm
            else:
                normal = [0, 0, 1]

            f.write(f"  facet normal {normal[0]:.4f} {normal[1]:.4f} {normal[2]:.4f}\\n")
            f.write("    outer loop\\n")
            for p in tri:
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\\n")
            f.write("    endloop\\n")
            f.write("  endfacet\\n")
        f.write("endsolid topology_optimized_mesh\\n")

    print(f"✅ Watertight 3D STL exported: {output_stl} ({len(triangles)} facets)")
    return output_stl


if __name__ == "__main__":
    # Demo 60x20 cantilever beam density field
    sample = np.random.uniform(0.1, 0.9, (60, 20))
    sample[sample < 0.45] = 0.0
    export_topology_to_stl(sample, output_stl="sample_bracket_3d.stl")
