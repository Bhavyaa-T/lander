import numpy as np
import matplotlib.pyplot as plt

m = 1
k = 1


t_max = 100
dt = 1.7
t_array = np.arange(0, t_max, dt)

x = np.zeros(len(t_array))
v = np.zeros(len(t_array))

v[1] = 1
x[0] = x[1] - dt * v[1]

for t in range(1, len(t_array) - 1):

    a = - k * x[t] / m
    x[t + 1] = 2 * x[t] - x[t-1] + dt * dt * a 
    v[t + 1] = (1 / dt) * (x[t + 1] - x[t]) 

#Analytic solution
y = []
for t in t_array:
    y.append(np.sin(t))

plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.grid()
plt.plot(t_array, x, label='x (m)')
plt.plot(t_array, v, label='v (m/s)')
plt.plot(t_array, y, label = 'sin(t)')
plt.legend()
plt.show()
