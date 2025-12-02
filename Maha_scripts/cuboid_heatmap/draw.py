# -------------------------------
# FILE: draw.py
# -------------------------------
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def draw_cuboid(ax, center, size, alpha=0.1):
    """ Draws a transparent cuboid on the 3D plot """
    cx, cy, cz = center
    sx, sy, sz = size

    x = [cx - sx/2, cx + sx/2]
    y = [cy - sy/2, cy + sy/2]
    z = [cz - sz/2, cz + sz/2]

    verts = [
        [ [x[0], y[0], z[0]], [x[1], y[0], z[0]], [x[1], y[1], z[0]], [x[0], y[1], z[0]] ],
        [ [x[0], y[0], z[1]], [x[1], y[0], z[1]], [x[1], y[1], z[1]], [x[0], y[1], z[1]] ],
        [ [x[0], y[0], z[0]], [x[1], y[0], z[0]], [x[1], y[0], z[1]], [x[0], y[0], z[1]] ],
        [ [x[0], y[1], z[0]], [x[1], y[1], z[0]], [x[1], y[1], z[1]], [x[0], y[1], z[1]] ],
        [ [x[0], y[0], z[0]], [x[0], y[1], z[0]], [x[0], y[1], z[1]], [x[0], y[0], z[1]] ],
        [ [x[1], y[0], z[0]], [x[1], y[1], z[0]], [x[1], y[1], z[1]], [x[1], y[0], z[1]] ]
    ]

    cuboid = Poly3DCollection(verts, facecolors='cyan', linewidths=1, edgecolors='gray', alpha=alpha)
    ax.add_collection3d(cuboid)
