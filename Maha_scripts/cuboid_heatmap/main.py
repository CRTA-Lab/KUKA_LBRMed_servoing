# -------------------------------
# FILE: main.py
# -------------------------------
import numpy as np
import matplotlib.pyplot as plt
from robot_utils import init_robot_chain, solve_ik, compute_manipulability, compute_joint_limit, compute_singularity_metric
from sampling import create_grid, sample_cone_orientations
from evaluation import evaluate_position
from draw import draw_cuboid

URDF_PATH = "iiwa14.urdf"
TARGET = (0.6, 0.0, 0.5) 
RADIUS_X = 0.2 # plus long en X
RADIUS_Y = 0.1
RADIUS_Z = 0.05
phantom_x = 0.06
phantom_y = 0.03
phantom_z = 0.03
STEP = 0.007 
AXIS7 = np.array([1, 0, 0])
HALF_ANGLE = np.deg2rad(30)
N_ORIENT = 1

NX = int((2 * RADIUS_X) / STEP) + 1
NY = int((2 * RADIUS_Y) / STEP) + 1
NZ = int((2 * RADIUS_Z) / STEP) + 1

if __name__ == "__main__":
    chain = init_robot_chain(URDF_PATH)
    grid = create_grid(TARGET, (RADIUS_X, RADIUS_Y, RADIUS_Z), STEP)
    orientations = sample_cone_orientations(AXIS7, HALF_ANGLE, N_ORIENT)
    scores = []
    for pos in grid:
        score = evaluate_position(chain, pos, orientations)
        scores.append(score)

    scores = np.array(scores)
    
    min_score = np.min(scores)
    max_score = np.max(scores)

    if max_score - min_score != 0:
        scores_normalized = (scores - min_score) / (max_score - min_score)
    else:
        scores_normalized = np.zeros_like(scores)
        
    scores = scores_normalized.reshape((NX, NY, NZ))



    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(grid[:, 0], grid[:, 1], grid[:, 2], c=scores.ravel(), cmap='turbo', s=20)
    fig.colorbar(sc, ax=ax, label='Score')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Scatter Plot of Full Score Cuboid')

    
    ik_solution = chain.inverse_kinematics(TARGET)
    chain.plot(ik_solution, ax, target=TARGET)

    draw_cuboid(ax, TARGET, size=(2*phantom_x, 2*phantom_y, 2*phantom_z), alpha=0.15)


    ax.set_xlim(-0.9 , 0.9)
    ax.set_ylim(-0.60, 0.60)
    ax.set_zlim(0, 2)
    
    frame_length = 0.05
    origin = np.array(TARGET)
    ax.quiver(*origin, frame_length, 0, 0, color='r', linewidth=2, arrow_length_ratio=0.2)
    ax.quiver(*origin, 0, frame_length, 0, color='g', linewidth=2, arrow_length_ratio=0.2)
    ax.quiver(*origin, 0, 0, frame_length, color='b', linewidth=2, arrow_length_ratio=0.2)
    ax.text(*(origin + [frame_length, 0, 0]), 'X', color='r')
    ax.text(*(origin + [0, frame_length, 0]), 'Y', color='g')
    ax.text(*(origin + [0, 0, frame_length]), 'Z', color='b')

    plt.show()


    

