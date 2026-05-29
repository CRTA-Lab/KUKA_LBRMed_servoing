import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

folder = '/home/racunalo/KUKA_LBRMed_servoing/logs/2026-05-26_16-52-10'

# Load data
probe_cols = ['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw','US_position']
phantom_cols = ['time','x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
data_probe = pd.read_csv(f'{folder}/position.log', header=None, names=probe_cols)
data_phantom = pd.read_csv(f'{folder}/phantom_position.log', header=None, names=phantom_cols)

t0 = data_phantom['time'].iat[0].astype(float)
data_probe['time'] = data_probe['time'].astype(float) - t0
data_phantom['time'] = data_phantom['time'].astype(float) - t0

# Discard probe data before first phantom timestamp
t_probe_min = data_phantom['time'].min()
#data_probe = data_probe[data_probe['time'] >= t_probe_min].reset_index(drop=True)
data_probe = data_probe[data_probe['time'] >= 0].reset_index(drop=True)

data_probe['z'] = data_probe['z'].astype(float) - data_probe['z'].iat[0].astype(float)
data_phantom['z'] = data_phantom['z'].astype(float) - data_phantom['z'].iat[0].astype(float)

data_probe['y'] = data_probe['y'].astype(float) - data_probe['y'].iat[0].astype(float)
data_phantom['y'] = data_phantom['y'].astype(float) - data_phantom['y'].iat[0].astype(float)

data_probe['x'] = data_probe['x'].astype(float) - data_probe['x'].iat[0].astype(float)
data_phantom['x'] = data_phantom['x'].astype(float) - data_phantom['x'].iat[0].astype(float)

data_probe['z'] *= 1000
data_phantom['z'] *= 1000

data_probe['y'] *= 1000
data_phantom['y'] *= -1000

data_probe['x'] *= -1000
data_phantom['x'] *= 1000



# Animation setup
fig, ax = plt.subplots(figsize=(24,14))
probe_line, = ax.plot([], [], 'b', label='probe position z')
phantom_line, = ax.plot([], [], 'b--', label='phantom position z')
ax.set_ylabel('position [mm]', color='blue', fontsize=18)
ax.set_xlabel('time [s]', fontsize=18)
ax.tick_params(axis='y', labelcolor='blue', labelsize=16)
ax.tick_params(axis='x', labelsize=16)
ax.set(ylim=(-5,40))
ax.grid(True,'both','both')
ax.legend(loc='upper left', fontsize=16)
plt.pause(5)


window = 15  # seconds

# Interpolate phantom z for probe times for smooth animation
times = data_probe['time'].values
phantom_interp = np.interp(times, data_phantom['time'].values, data_phantom['z'].values)

# Add y data lines
probe_line_y, = ax.plot([], [], 'r', label='probe position y')
phantom_line_y, = ax.plot([], [], 'r--', label='phantom position y')
# Interpolate phantom y for probe times
phantom_interp_y = np.interp(times, data_phantom['time'].values, data_phantom['y'].values)

# Update legend to include y
handles, labels = ax.get_legend_handles_labels()
ax.legend(handles, labels, loc='upper left', fontsize=16)

# Animation function
def animate(i):
    t_probe = data_probe[data_probe['time'].between(i/10, i/10+10)]['time'].values
    t_phantom = data_phantom[data_phantom['time'].between(i/10, i/10+10)]['time'].values
    z_probe = data_probe[data_probe['time'].between(i/10, i/10+10)]['z'].values
    z_phantom = data_phantom[data_phantom['time'].between(i/10, i/10+10)]['y'].values
    y_probe = data_probe[data_probe['time'].between(i/10, i/10+10)]['x'].values
    y_phantom = data_phantom[data_phantom['time'].between(i/10, i/10+10)]['z'].values
    if len(t_probe) == 0:
        return probe_line, phantom_line, probe_line_y, phantom_line_y
    
    probe_line.set_data(t_probe, z_probe)
    phantom_line.set_data(t_phantom, z_phantom)
    probe_line_y.set_data(t_probe, y_probe)
    phantom_line_y.set_data(t_phantom, y_phantom)
    ax.set_xlim(t_probe.min(), t_probe.max())
    return probe_line, phantom_line, probe_line_y, phantom_line_y

ani = FuncAnimation(fig, animate, frames=10000, interval=100, blit=True)
plt.show()
