import numpy as np
import matplotlib.pyplot as plt
import sys
import os

assert len(sys.argv) == 3 , "python plot_fancy.py <path_to_data> <plot_path>"

data = np.loadtxt(sys.argv[1])

fig, ax = plt.subplots(2,1, figsize=(10, 5), sharex=True)

data_degr = data *180/np.pi
time = np.arange(len(data_degr))

ax[0].plot(time, data_degr[:,0], label="measured", color="C1")
ax[0].plot(time, data_degr[:,1], label="desired", color="C0")
ax[1].plot(time, data_degr[:,2], label="error", color="C3")

ax[0].set_ylabel("measured vs desired [deg]")
ax[1].set_ylabel("error [deg]")
ax[1].set_xlabel("time step")
ax[1].set_xlim(0)

ax[0].legend()
ax[1].legend()

plt.tight_layout()

plt.savefig(sys.argv[2], bbox_inches='tight')
os.remove(sys.argv[1])
