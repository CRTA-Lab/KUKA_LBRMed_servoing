import numpy as np

def dh_matrix(theta, d, a, alpha):
    """Returns the individual transformation matrix using DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

def forward_kinematics_and_jacobian(thetas):
    """Computes forward kinematics, Jacobian, and prints all transformation matrices."""
    dh_params = [
        [thetas[0], 0.36, 0.0, -np.pi/2],
        [thetas[1], 0.0, 0.0, np.pi/2],
        [thetas[2], 0.42, 0.0, np.pi/2],
        [thetas[3], 0.0, 0.0, -np.pi/2],
        [thetas[4], 0.4, 0.0, -np.pi/2],
        [thetas[5], 0.0, 0.0, np.pi/2],
        [thetas[6], 0.126, 0.0, 0]
    ]

    T = np.eye(4)
    print("Homogeneous Transformation Matrices:\n")

    for i, (theta, d, a, alpha) in enumerate(dh_params):
        T_i = dh_matrix(theta, d, a, alpha)
        T = np.dot(T, T_i)
        print(f"T_{i+1}:\n{np.round(T, 3)}\n")

    print("Final End-Effector Pose (T_7):\n", np.round(T, 3))

    d3 = dh_params[2][1]
    d5 = dh_params[4][1] 

    thetas = [row[0] for row in dh_params]
    c = np.cos
    s = np.sin
    c1, c2, c3, c4, c5, c6 = [c(t) for t in thetas[:6]]
    s1, s2, s3, s4, s5, s6 = [s(t) for t in thetas[:6]]

    J51 = d3 * s4 * (c2 * c4 + c3 * s2 * s4) - d3 * c4 * (c2 * s4 - c3 * c4 * s2)

    J = np.array([
        [c2*s4 - c3*c4*s2,      c4*s3,       s4,    0,  0,   -s5,     c5*s6],
        [s2*s3,                c3,           0,    -1,  0,    c5,     s5*s6],
        [c2*s4 + c3*s2*s4,    -s3*s4,        c4,    0,  1,     0,      c6],
        [d3*c4*s2*s3,         d3*c3*c4,      0,     0,  0, -d5*c5, -d5*s5*s6],
        [J51,                -d3*s3,         0,     0,  0, -d5*s5,  d5*c5*s6],
        [-d3*s2*s3*s4,       -d3*c3*s4,      0,     0,  0,     0,      0]
    ])
    print("The Jacobian matrix:\n", np.round(J,3))
    return J,T

def check_singularity(J, threshold=1e-4):
    """Checks if the robot is in or near a singular configuration."""
    singular_values = np.linalg.svd(J, compute_uv=False)
    min_singular = np.min(singular_values)
    print("\nSingular Values of Jacobian:\n", np.round(singular_values, 5))
    if min_singular < threshold:
        print("\n Robot is in or near a SINGULAR configuration (min sigma =", min_singular, ")")
    else:
        print("\n Robot is NOT in a singular configuration (min sigma =", min_singular, ")")

# Example usage
theta_values =  [0, 0, 0, 0, 0, 0, 0]
jacobian, end_effector_pose = forward_kinematics_and_jacobian(theta_values)
check_singularity(jacobian)