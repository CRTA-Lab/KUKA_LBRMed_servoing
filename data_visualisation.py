import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

folder = '/home/racunalo/KUKA_LBRMed_servoing/logs/2026-05-26_10-16-43' #dobar vertikalni
#folder = '/home/racunalo/KUKA_LBRMed_servoing/logs/2025-09-09_10-28-59' #dobar horizontalni
#folder = '/home/racunalo/KUKA_LBRMed_servoing/logs/2025-09-09_11-00-14'

data_probe = pd.read_csv(folder + '/position.log', header=None, names=['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw','US_position'])
data_phantom = pd.read_csv(folder + '/phantom_position.log', header=None, names=['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
data_force = pd.read_csv(folder + '/force_compensation.log', header=None, names=['time','fx', 'fy', 'fz', 'tx', 'ty', 'tz'])

t0 = data_phantom['time'].iat[0].astype(float)
data_force['time'] = data_force['time'].astype(float) - t0
data_probe['time'] = data_probe['time'].astype(float) - t0
data_phantom['time'] = data_phantom['time'].astype(float) - t0
data_probe = data_probe[data_probe['time'] >= 0].reset_index(drop=True)

data_probe['z'] = data_probe['z'].astype(float) - data_probe['z'].iat[0].astype(float)
data_phantom['z'] = data_phantom['z'].astype(float) - data_phantom['z'].iat[0].astype(float)

data_probe['x'] = data_probe['y'].astype(float) - data_probe['y'].iat[0].astype(float) + 2.5/1000
data_phantom['y'] = data_phantom['x'].astype(float) - data_phantom['x'].iat[0].astype(float)

data_probe['x'] *= 1000
data_phantom['y'] *= 1000

data_probe['z'] *= 1000
data_phantom['z'] *= 1000

# data_probe.sort_values(by='time', inplace=True)
# data_phantom.sort_values(by='time', inplace=True)

# arr = np.arange(0,10,0.01)
# plt.plot(arr,np.cos(arr)*((arr/np.pi)%1>0.5))
# plt.show()


# plt.plot(data_force['time'],data_force['fy'], label='force y')
# plt.plot(data_force['time'],data_force['fz'], label='force z')
# plt.legend()
# plt.grid(True,'both','both')
# plt.show()

fig,ax1 = plt.subplots(figsize=(16,9))
ax1.plot(data_probe['time'],data_probe['x'],'r', label='probe position x')
ax1.plot(data_phantom['time'],data_phantom['y'], 'r--', label='phantom position x')
ax1.plot(data_probe['time'],data_probe['z'],'b', label='probe position z')
ax1.plot(data_phantom['time'],data_phantom['z'], 'b--', label='phantom position z')
ax1.set_ylabel('position [mm]', color='blue', fontsize=18)
ax1.set_xlabel('time [s]', fontsize=18)
ax1.tick_params(axis='y', labelcolor='blue', labelsize=16)
ax1.tick_params(axis='x', labelsize=16)
#ax1.set(xlim=(12.5,24.5))
#ax1.set(xlim=(25.5,37.5))
#ax1.set(xlim=(39,51))
#ax1.set(ylim=(-3,16))
ax1.grid(True,'both','both')
ax1.legend(loc='upper left', fontsize=16)

# ax2 = ax1.twinx()
# ax2.plot(data_force['time'],data_force['fy'], 'black', label='$F_y$')
# ax2.plot(data_force['time'],data_force['fz'], 'k--', label='$F_z$')
# ax2.set_ylabel('force [N]', color='black', fontsize=18)
# ax2.tick_params(axis='y', labelcolor='black', labelsize=16)
# ax2.legend(loc='upper right', fontsize=16)
# ax2.set(ylim=(-4,12))
plt.show()
#fig.savefig('/home/racunalo/KUKA_LBRMed_servoing/graphs/horisontal_motion.png', dpi=300,bbox_inches='tight',pad_inches=0.0)

