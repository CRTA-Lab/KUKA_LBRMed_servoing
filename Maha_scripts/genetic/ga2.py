from numpy.random import randint
from numpy.random import rand
import math
import random
from ikpy.chain import Chain
from ikpy.link import OriginLink
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



robot_chain = Chain.from_urdf_file("iiwa14.urdf", base_elements=["iiwa_link_0"])

def solve_ik(target):
    try:
        ik_solution = robot_chain.inverse_kinematics(target)
        return ik_solution[1:8]
    except Exception:
        return None


#--------------------------------------------------------------


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

#---------------------------------------------------------------


def compute_manipulability(joint_values):
    J = get_jacobian(joint_values)  # J should be 6x7
    JJ_T = np.dot(J, J.T) 
    try:
        manipulability = np.sqrt(np.linalg.det(JJ_T))
    except np.linalg.LinAlgError:
        manipulability = 0.0
    return manipulability

#----------------------------------------------------------------


JOINT_LIMITS = [
    (-2.967, 2.967),  # Joint 1
    (-2.094, 2.094),  # Joint 2
    (-2.967, 2.967),  # Joint 3
    (-2.094, 2.094),  # Joint 4
    (-2.967, 2.967),  # Joint 5
    (-2.094, 2.094),  # Joint 6
    (-3.054, 3.054),  # Joint 7
]


def compute_joint_limit_distance(joint_values):
    # If there's an extra joint value, skip it
    if len(joint_values) > len(JOINT_LIMITS):
        joint_values = joint_values[1:]

    distances = []
    for idx, (val, (jmin, jmax)) in enumerate(zip(joint_values, JOINT_LIMITS)):
        dist_to_min = abs(val - jmin)
        dist_to_max = abs(jmax - val)

        joint_range = jmax - jmin
        norm_dist = min(dist_to_min, dist_to_max) / joint_range

        # Introduce sinusoidal modulation
        sinusoidal_component = np.sin(2 * np.pi * (val - jmin) / joint_range)
        modulated_norm_dist = norm_dist * (0.5 + 0.5 * sinusoidal_component)

        # Normalize to ensure the highest value is 1 and the lowest is 0
        normalized_modulated_dist = (modulated_norm_dist - 0) / (1 - 0)
        normalized_modulated_dist = max(0, min(normalized_modulated_dist, 1))

        if normalized_modulated_dist < 1e-2:
            return 0.0

        distances.append(normalized_modulated_dist)
    return sum(distances) / len(distances)


#----------------------------------------------------------------

def compute_singularity_metric(joint_values):
    J = get_jacobian(joint_values)
    cond_number = np.linalg.cond(J, 2)
    return 1.0/min(1000, cond_number)

#----------------------------------------------------------------


def objective(tcp_target):
    w2, w3, w4 = 1.0, 1.5, 2.0  # weights
    ik_solution = solve_ik(tcp_target)
    if ik_solution is None:
        return float('inf')
    total_manipulability = compute_manipulability(ik_solution)
    total_joint_limit_dist = compute_joint_limit_distance(ik_solution)
    total_singularity_metric = compute_singularity_metric(ik_solution)
    score = (
        -total_manipulability * w2
        -total_joint_limit_dist * w3
        -total_singularity_metric * w4
    )
    return score


#---------------------------------------------------------------

def decode(bounds, n_bits, bitstring):
    decoded = []
    largest = 2**n_bits
    for i in range (len(bounds)):
        start, end = i * n_bits, (i + 1) * n_bits
        substring = bitstring[start:end]
        chars = ''.join([str(s) for s in substring])
        integer = int(chars, 2)
        value = bounds[i][0] + (integer / largest) * (bounds[i][1] - bounds[i][0])
        decoded.append(value)
    return decoded

#---------------------------------------------------------------

def selection(population, scores, k=3):
    selection_ix = randint(len(population))
    for ix in randint(0, len(population), k-1):
        if scores[ix] < scores[selection_ix]:
            selection_ix = ix
    return population[selection_ix]

#----------------------------------------------------------------

def crossover(p1, p2, r_cross):
    c1, c2 = p1.copy(), p2.copy()
    if rand() < r_cross:
        pt = randint(1, len(p1)-1)
        c1 = p1[:pt] + p2[pt:]
        c2 = p2[:pt] + p1[pt:]
    return [c1, c2]

#----------------------------------------------------------------

def mutation(bitstring, r_mut):
    rand = random.random
    for i in range(len(bitstring)):
        if rand() < r_mut:
            bitstring[i] = 1 - bitstring[i]  # Flip the bit
    return bitstring

#----------------------------------------------------------------

def genetic_algorithm(objective, bounds, n_bits, n_iter, n_pop, r_cross, r_mut):
    all_positions = []
    all_scores = []
    all_manipulability = []
    all_joint_limit_dist = []
    all_singularity = []
    population = [randint(0, 2, n_bits * len(bounds)).tolist() for _ in range(n_pop)]
    best, best_eval = 0, objective(decode(bounds, n_bits, population[0]))
    for gen in range(n_iter):
        decoded = [decode(bounds, n_bits, p) for p in population]
        scores = []
        manipulabilities = []
        joint_limit_dists = []
        singularities = []
        for d in decoded:
            ik_solution = solve_ik(d)
            if ik_solution is None:
                score = float('inf')
                m = 0
                jld = 0
                s = 0
            else:
                m = compute_manipulability(ik_solution)
                jld = compute_joint_limit_distance(ik_solution)
                s = compute_singularity_metric(ik_solution)
                score = (
                    -m * 1.0
                    -jld * 1.5
                    -s * 2.0
                )
            scores.append(score)
            manipulabilities.append(m)
            joint_limit_dists.append(jld)
            singularities.append(s)
        all_positions.extend(decoded)
        all_scores.extend(scores)
        all_manipulability.extend(manipulabilities)
        all_joint_limit_dist.extend(joint_limit_dists)
        all_singularity.extend(singularities)
        for i in range(n_pop):
            if scores[i] < best_eval:
                best, best_eval = population[i], scores[i]
                print(f"Gen {gen}, New Best: {best_eval}, Params: {decoded[i]}")
        selected = [selection(population, scores) for _ in range(n_pop)]
        children = []
        for i in range(0, n_pop, 2):
            p1, p2 = selected[i], selected[i + 1]
            for c in crossover(p1, p2, r_cross):
                mutation(c, r_mut)
                children.append(c)
        population = children
    best_decoded = decode(bounds, n_bits, best)
    return best, best_eval, all_positions, all_scores, all_manipulability, all_joint_limit_dist, all_singularity

#----------------------------------------------------------------

bounds = [
    (0, 0.6),  # target_x (meters)
    (-0.45, 0.45), # target_y (meters)
    (0, 1.6),  # target_z (meters)
]
n_bits = 16
n_iter = 100
n_pop = 100
r_cross = 0.9
r_mut = 1.0 / (n_bits * len(bounds))

best, best_eval, all_positions, all_scores, all_manipulability, all_joint_limit_dist, all_singularity = genetic_algorithm(objective, bounds, n_bits, n_iter, n_pop, r_cross, r_mut)
best_decoded = decode(bounds, n_bits, best)

print("Best target position:", best_decoded)
print("Best objective score:", best_eval)
#----------------------------------------------------------------

np.save('all_positions.npy', np.array(all_positions))
np.save('all_scores.npy', np.array(all_scores))


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

all_positions = np.array(all_positions)
all_scores = np.array(all_scores)

norm_scores = (all_scores - np.min(all_scores)) / (np.max(all_scores) - np.min(all_scores) + 1e-8)

sc = ax.scatter(all_positions[:, 0], all_positions[:, 1], all_positions[:, 2], c=norm_scores, cmap='viridis', s=10, label='Target Positions')

ax.scatter(best_decoded[0], best_decoded[1], best_decoded[2], color='red', s=120, label='Best Target Position')

ax.scatter(0, 0, 0, color='blue', s=120, label='Base Position')

plt.colorbar(sc, ax=ax, label='Normalized Objective Score')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title('Heatmap of Target Positions (Genetic Algorithm)')
plt.show()

all_scores = np.array(all_scores)
all_manipulability = np.array(all_manipulability)
all_joint_limit_dist = np.array(all_joint_limit_dist)
all_singularity = np.array(all_singularity)

plt.figure(figsize=(8, 6))
plt.scatter(all_manipulability, all_scores, c='purple', alpha=0.6, label='Manipulability')
plt.scatter(all_joint_limit_dist, all_scores, c='green', alpha=0.6, label='Joint Limit Distance')
plt.scatter(all_singularity, all_scores, c='orange', alpha=0.6, label='Singularity Metric')
plt.xlabel('Metric Value')
plt.ylabel('Objective Score')
plt.title('Objective Score vs Metrics')
plt.legend()
plt.grid(True)
plt.show()





