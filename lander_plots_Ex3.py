import numpy as np
import matplotlib.pyplot as plt

# Load trajectory data: columns [time, altitude, vertical_velocity]
results = np.loadtxt('trajectories.txt')
results_2 = np.loadtxt('trajectories_disturb.txt')

t = results[:, 0]
h = results[:, 1]

t_1 = results_2[:, 0]
h_1 = results_2[:, 1]

plt.figure(1)
plt.clf()
plt.grid()
plt.xlabel('time (s)')
plt.ylabel('altitude h (m)')

plt.plot(t, h)

plt.legend()
plt.show()

plt.figure(2)
plt.clf()
plt.grid()
plt.xlabel('time (s)')
plt.ylabel('altitude h (m)')
plt.plot(t_1, h_1)
plt.legend()
plt.show()
