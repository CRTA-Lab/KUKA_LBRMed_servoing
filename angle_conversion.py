import numpy as np
from scipy.spatial.transform import Rotation

input_angles = [130.63,-89.67,98.38]
rot = Rotation.from_euler('ZYX', input_angles, degrees=True)
output_angles = rot.as_euler('xyz', degrees=False)

print(output_angles)