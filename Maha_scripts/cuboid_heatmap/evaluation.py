# -------------------------------
# FILE: evaluation.py
# -------------------------------
"""
Evaluate score for each grid position and set of orientations.
"""
import numpy as np
from robot_utils import solve_ik, compute_manipulability, \
                           compute_joint_limit, compute_singularity_metric

 

def evaluate_position(chain, pos, orientations):
    scores = []
    for R in orientations:
        q = solve_ik(chain, pos, R)
        if q is None:
            scores.append(-100)
        else:
            # calcul des trois métriques
            m = compute_manipulability(q)
            j = compute_joint_limit(q)
            s = compute_singularity_metric(q)

            score = m + 1.5*j + 2.0*s
            scores.append(score)
    return np.mean(scores)

