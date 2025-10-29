import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, sosfilt, sosfilt_zi

folder = '/home/racunalo/KUKA_LBRMed_servoing/logs/2025-09-01_09-52-08'

data_probe = pd.read_csv(folder + '/position.log', header=None, names=['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw','x_img'])
#data_phantom = pd.read_csv(folder + '/phantom_position.log', header=None, names=['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
data_force = pd.read_csv(folder + '/force_compensation.log', header=None, names=['time','fx', 'fy', 'fz', 'tx', 'ty', 'tz'])



data_probe['z'] *= 1000

position = data_probe['x_img']

filt = butter(5, 1, 'high', output='sos',fs=18)
filtered = sosfilt(filt,position)

filt = butter(5, [1,8], 'bandpass', output='sos',fs=18)
filtered_bandpass = sosfilt(filt, position)

plt.plot(data_probe['time'],data_probe['x_img'])
plt.plot(data_force['time'],data_force['fx'])
#plt.plot(data_probe['time'],filtered)
#plt.plot(data_probe['time'],filtered_bandpass)
plt.grid(True,'both','both')
plt.show()

