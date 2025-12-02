# -------------------------------
# FILE: sampling.py
# -------------------------------
"""
Generate spatial grid and cone orientations for axis 7.
"""
import numpy as np

def create_grid(center, radius, step):
    x0, y0, z0 = center
    rx, ry, rz = radius
    x_vals = np.arange(x0 - rx, x0 + rx + 1e-9, step)
    y_vals = np.arange(y0 - ry, y0 + ry + 1e-9, step)
    z_vals = np.arange(z0 - rz, z0 + rz + 1e-9, step)
    X, Y, Z = np.meshgrid(x_vals, y_vals, z_vals, indexing='xy')
    grid = np.vstack([X.ravel(), Y.ravel(), Z.ravel()]).T
    return grid



def sample_cone_orientations(axis, half_angle, n_samples):
    axis = axis / np.linalg.norm(axis)
    # Vecteur arbitraire non-aligné pour construire la base
    v0 = np.array([1.0, 0.0, 0.0])
    if np.allclose(v0, axis):
        v0 = np.array([0.0, 1.0, 0.0])
    # Base orthonormée (u, w, axis)
    u = np.cross(axis, v0)
    u /= np.linalg.norm(u)
    w = np.cross(axis, u)
    
    orientations = []
    for theta in np.linspace(0, 2*np.pi, n_samples, endpoint=False):
        dir_cone = (
            np.cos(half_angle) * axis +
            np.sin(half_angle) * (np.cos(theta) * u + np.sin(theta) * w)
        )
        z_new = dir_cone
        v1 = u
        x_new = v1 - np.dot(v1, z_new) * z_new
        x_new /= np.linalg.norm(x_new)
        y_new = np.cross(z_new, x_new)
        R = np.column_stack((x_new, y_new, z_new))
        orientations.append(R)
    return orientations

