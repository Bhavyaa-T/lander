import numpy as np
import matplotlib.pyplot as plt


# let r_p = 10 km above the surface
# For an elliptical orbit, 0 < e < 1. We can choose e = 0.5
# this lets us solve for v_p = sqrt(G * M * (1 + e) / r_p) 

# mass, gravitational constant, mass of Mars, radius of Mars
m = 1
G = 6.67430e-11
M = 6.42e23
R = 3.39e6

# simulation time, timestep and time
t_max = 10000
dt = 0.1
t_array = np.arange(0, t_max, dt)


position = np.zeros((len(t_array), 3))
velocity = np.zeros((len(t_array), 3))

#initial position and velocity
position[0] = [R + 1e4, 0, 0]
velocity[0] = [0, 4350 ,0]

#initial step
position[1] = position[0] + dt * velocity[0]

#Verlet integration
for t in range(1, len(t_array) - 1):

    a = - G * M * position[t] / (np.linalg.norm(position[t]))**3
    position[t + 1] = 2 * position[t] - position[t-1] + dt * dt * a 
    velocity[t + 1] = (1 / dt) * (position[t + 1] - position[t]) 


# plot the position-time graph (x component)
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.ylabel('x (m)')
plt.grid()
plt.plot(t_array, position[:, 0], label='x (m)')
plt.legend()
plt.show()

# plot the x-y trajectory to show the orbit
plt.figure(3)
plt.clf()
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.axis('equal')
plt.grid()
plt.plot(position[:, 0], position[:, 1], label='Orbit trajectory')
plt.legend()
plt.show()
