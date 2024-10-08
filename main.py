import numpy as np
import matplotlib.pyplot as plt

# Define time duration
t_f = 2  # seconds
timesteps_per_second = 100
time = np.linspace(0, t_f, t_f * timesteps_per_second)

# Given constants for boundary conditions
theta_0 = 0
theta_f = 160
theta_dot_0 = 0
theta_dot_f = 0
theta_dotdot_0 = 0
theta_dotdot_f = 0

# Coefficients for the matrix
a1 = t_f**3
b1 = t_f**4
c1 = t_f**5
a2 = 3 * (t_f**2)
b2 = 4 * (t_f**3)
c2 = 5 * (t_f**4)
a3 = 6 * t_f
b3 = 12 * (t_f**2)
c3 = 20 * (t_f**3)

# Constants matrix
d1 = theta_f - theta_0 - theta_dot_0 * t_f - 0.5 * theta_dotdot_0 * (t_f**2)
d2 = theta_dot_f - theta_dot_0 - theta_dotdot_0 * t_f
d3 = theta_dotdot_f - theta_dotdot_0

# Matrix A and B
A = np.array([[a1, b1, c1],
              [a2, b2, c2],
              [a3, b3, c3]])

B = np.array([d1, d2, d3])

# Initial coefficients
a_0 = theta_0
a_1 = theta_dot_0
a_2 = theta_dotdot_0 / 2  # Divided by 2 because it's part of the quadratic term

# Solve for a_3, a_4, a_5
solution = np.linalg.solve(A, B)
a_3 = solution[0]
a_4 = solution[1]
a_5 = solution[2]

# Define the fifth-degree polynomial and its derivatives
theta = a_0 + a_1 * time + a_2 * time**2 + a_3 * time**3 + a_4 * time**4 + a_5 * time**5
theta_dot = a_1 + 2 * a_2 * time + 3 * a_3 * time**2 + 4 * a_4 * time**3 + 5 * a_5 * time**4
theta_dotdot = 2 * a_2 + 6 * a_3 * time + 12 * a_4 * time**2 + 20 * a_5 * time**3

# Plot the polynomial and its derivatives
plt.figure(figsize=(10, 6))

plt.plot(time, theta, label='Position (theta)', color='blue')
plt.plot(time, theta_dot, label='Velocity (theta_dot)', color='green')
plt.plot(time, theta_dotdot, label='Acceleration (theta_dotdot)', color='red')

# Labeling the plot
plt.title('Fifth-degree Polynomial and its Derivatives')
plt.xlabel('Time (seconds)')
plt.ylabel('Magnitude')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
