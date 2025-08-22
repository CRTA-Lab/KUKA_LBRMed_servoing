import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


data_probe = pd.read_csv('/home/racunalo/KUKA_LBRMed_servoing/logs/2025-08-21_13-29-25/position.log', header=None, names=['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
data_phantom = pd.read_csv('/home/racunalo/KUKA_LBRMed_servoing/logs/2025-08-21_13-29-25/phantom_position.log', header=None, names=['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])


data_probe['z']= data_probe['z'].astype(float) - data_probe['z'][0].astype(float)
data_phantom['z']= data_phantom['z'].astype(float) - data_phantom['z'][0].astype(float)

data_probe['z'] *= 1000
data_phantom['z'] *= 1000

# data_probe.sort_values(by='time', inplace=True)
# data_phantom.sort_values(by='time', inplace=True)

# arr = np.arange(0,10,0.01)
# plt.plot(arr,np.cos(arr)*((arr/np.pi)%1>0.5))
# plt.show()

plt.plot(data_probe['time'],data_probe['z'])
plt.plot(data_phantom['time'],data_phantom['z'])
plt.show()

