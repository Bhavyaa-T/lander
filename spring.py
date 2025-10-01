# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt



# mass, gravitational constant, mass of Mars
G = 6.67430e-11
M = 6.42e23
R = 3.39e6

# let r_p = 10 km above the surface
# For an elliptical orbit, 0 < e < 1. We can choose e = 0.5
# this lets us solve for v_p = sqrt(G * M * (1 + e) / r_p) 

# initial position and velocity as vectors
position = np.array([R + 1e4, 0, 0])
velocity = np.array([0, 4350, 0])  

# simulation time, timestep and time
t_max = 10000
dt = 0.1
t_array = np.arange(0, t_max, dt)


# initialise empty lists to record trajectories
position_list = []
velocity_list = []


# Euler integration for vector position and velocity

for t in t_array:
    # append current state to trajectories
    position_list.append(position.copy())
    velocity_list.append(velocity.copy())

    # calculate new position and velocity
    a = -G * M * position / (np.linalg.norm(position))**3
    position = position + dt * velocity
    velocity = velocity + dt * a


# convert trajectory lists into arrays
position_array = np.array(position_list)
velocity_array = np.array(velocity_list)



# plot the position-time graph (x component)
plt.figure(1)
plt.clf()
plt.xlabel('time (s)')
plt.ylabel('x (m)')
plt.grid()
plt.plot(t_array, position_array[:, 0], label='x (m)')
plt.legend()
plt.show()


# plot the x-y trajectory to show the orbit
plt.figure(3)
plt.clf()
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.axis('equal')
plt.grid()
plt.plot(position_array[:, 0], position_array[:, 1], label='Orbit trajectory')
plt.legend()
plt.show()

