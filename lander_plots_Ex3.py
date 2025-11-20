import numpy as np
import matplotlib.pyplot as plt

# Load trajectory data: columns [time, altitude, vertical_velocity]
results = np.loadtxt('trajectories.txt')

t = results[:, 0]
h = results[:, 1]
v = results[:, 2]

plt.figure(1)
plt.clf()
plt.grid()
plt.xlabel('time (s)')
plt.ylabel('altitude h (m)')

# Actual vertical velocity
plt.plot(t, h)

plt.legend()
plt.show()
