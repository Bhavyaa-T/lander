import numpy as np
import matplotlib.pyplot as plt

# mass, spring constant, initial position and velocity
m = 1
k = 1

# simulation time, timestep and time
t_max = 100
dt = 0.1
t_array = np.arange(0, t_max, dt)


x = np.zeros(len(t_array))
v = np.zeros(len(t_array))

#initial position and velocity
x[0] = 0
v[0] = 1

#initial step
x[1] = x[0] + dt * v[0]

#Verlet integration
for t in range(1, len(t_array) - 1):

    a = - k * x[t] / m
    x[t + 1] = 2 * x[t] - x[t-1] + dt * dt * a 
    v[t + 1] = (1 / dt) * (x[t + 1] - x[t]) 

#Analytic solution
y = []
times = np.arange(0, t_max, 0.1)
for t in times:
    y.append(np.sin(t))

# plot the position-time graph
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x, label='x (m)')
plt.plot(t_array, v, label='v (m/s)')
plt.plot(times, y, label = 'sin(t)')
plt.legend()
plt.show()

#from testing, the Verlet method becomes unstable at dt = 2