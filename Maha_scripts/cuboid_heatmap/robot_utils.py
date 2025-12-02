# -------------------------------
# FILE: robot_utils.py
# -------------------------------
"""
Helpers for robot initialization, IK solving, and metric computations.
"""
import numpy as np
from ikpy.chain import Chain
import math

# Load robot chain from URDF
def init_robot_chain(urdf_path):
    robot_chain = Chain.from_urdf_file(urdf_path, base_elements=["iiwa_link_0"])
    return robot_chain

# Solve inverse kinematics for given position+orientation
def solve_ik(chain, target, orientation):
    try:
        ik_solution = chain.inverse_kinematics(target, orientation)
        return ik_solution[1:8]
    except Exception:
        return None
    
# Compute jacobian
def get_jacobian(joint_values):

    thetas = joint_values[:6]
    c = np.cos
    s = np.sin
    c1, c2, c3, c4, c5, c6 = [c(t) for t in thetas]
    s1, s2, s3, s4, s5, s6 = [s(t) for t in thetas]
    d3, d5 = 0.42, 0.4  

    J51 = d3 * s4 * (c2 * c4 + c3 * s2 * s4) - d3 * c4 * (c2 * s4 - c3 * c4 * s2)
    J = np.array([
        [c2*s4 - c3*c4*s2, c4*s3, s4, 0, 0, -s5, c5*s6],
        [s2*s3, c3, 0, -1, 0, c5, s5*s6],
        [c2*s4 + c3*s2*s4, -s3*s4, c4, 0, 1, 0, c6],
        [d3*c4*s2*s3, d3*c3*c4, 0, 0, 0, -d5*c5, -d5*s5*s6],
        [J51, -d3*s3, 0, 0, 0, -d5*s5, d5*c5*s6],
        [-d3*s2*s3*s4, -d3*c3*s4, 0, 0, 0, 0, 0]
    ])
    return J

# Compute manipulability index
def compute_manipulability(joints):
    J = get_jacobian(joints)
    JJ_T = np.dot(J, J.T) 
    try:
        manipulability = np.sqrt(np.linalg.det(JJ_T))
    except np.linalg.LinAlgError:
        manipulability = 0.0
    manipulability = np.clip(manipulability, 0.0, 0.17)
    normalized_mani = manipulability / 0.17
    return normalized_mani


# Compute distance to joint limits
JOINT_LIMITS = [
    (-2.967, 2.967),  # Joint 1
    (-2.094, 2.094),  # Joint 2
    (-2.967, 2.967),  # Joint 3
    (-2.094, 2.094),  # Joint 4
    (-2.967, 2.967),  # Joint 5
    (-2.094, 2.094),  # Joint 6
    (-3.054, 3.054),  # Joint 7
]
safe_margin_ratio = 0.7 

def compute_joint_limit(joint_values):
    if len(joint_values) > len(JOINT_LIMITS):
        joint_values = joint_values[1:]

    scores = []

    for val, (jmin, jmax) in zip(joint_values, JOINT_LIMITS):
        joint_range = jmax - jmin
        safe_min = jmin + (1 - safe_margin_ratio) / 2 * joint_range
        safe_max = jmax - (1 - safe_margin_ratio) / 2 * joint_range

        if safe_min <= val <= safe_max:
            score = 1.0
        else:
            if val < safe_min:
                x = (safe_min - val) / (safe_min - jmin)
            else:
                x = (val - safe_max) / (jmax - safe_max)
            x = min(max(x, 0.0), 1.0)
            score = 0.5 * (1 + math.cos(x * math.pi))
        scores.append(score)
    return sum(scores) / len(scores)


# Compute singularity metric
min_score=0.001 
max_score=0.17

def compute_singularity_metric(joint_values):
    J = get_jacobian(joint_values)
    cond_number = np.linalg.cond(J, 2)
    raw_score = 1.0 / min(1000, cond_number)
    raw_score = np.clip(raw_score, min_score, max_score)
    normalized_score = (raw_score - min_score) / (max_score - min_score)
    return normalized_score








