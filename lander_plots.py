import numpy as np
import matplotlib.pyplot as plt

# Load trajectory data: columns [time, altitude, vertical_velocity]
results = np.loadtxt('trajectories.txt')

t = results[:, 0]
h = results[:, 1]
v = results[:, 2]

# Figure 1: Vertical velocity vs altitude, limited to 0 < h < 1000 and reversed x-axis

mask = (h > 0) & (h < 1000)
h_sel = h[mask]
v_sel = v[mask]

plt.figure(1)
plt.clf()
plt.grid()
plt.xlabel('altitude h (m)')
plt.ylabel('vertical velocity v (m/s)')

# Actual vertical velocity
plt.plot(h_sel, v_sel, label='v (m/s)')

# Target descent rate: v_target = -(0.5 + 0.015 * h)
v_target = -(0.5 + 0.015 * h_sel)
plt.plot(h_sel, v_target, 'r--', label='v_target (m/s)')

# Limit altitude range and reverse x-axis so h=0 is on the right
plt.xlim(1000, 0)
plt.legend()
plt.show()

# Figure 2: Vertical velocity vs time (unchanged)
plt.figure(2)
plt.clf()
plt.xlabel('time (s)')
plt.ylabel('vertical velocity v (m/s)')
plt.grid()
plt.plot(t, v, label='v (m/s)')
plt.legend()
plt.show()
