import numpy as np
import math
import matplotlib.pyplot as plt
from IK import inverse_kinematics  # Ensure this function is accessible in your environment

# Define the function to plot the manipulator given the joint angles
def plot_manipulator(theta_angles, link_lengths, ax, title):
    x = [0]
    y = [0]
    angle_sum = 0

    # Calculate x, y positions for each joint based on the link lengths and angles
    for i in range(3):
        angle_sum += theta_angles[i]
        x.append(x[-1] + link_lengths[i] * np.cos(angle_sum))
        y.append(y[-1] + link_lengths[i] * np.sin(angle_sum))

    # Plot the manipulator links
    ax.plot(x[0:2], y[0:2], color='red', linewidth=2)
    ax.plot(x[1:3], y[1:3], color='green', linewidth=2)
    ax.plot(x[2:4], y[2:4], color='blue', linewidth=2)

    # Plot the joints
    ax.scatter(x, y, color='black')
    ax.quiver(x[-1], y[-1], np.cos(angle_sum), np.sin(angle_sum), scale=10, color='red',
              width=0.005)  # End-effector direction

    # Set titles and axis limits
    ax.set_title(title)
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.axis('equal')
    ax.grid(True)


# Given constants for boundary conditions for three joints
x_desired = -8
y_desired = 20
theta_desired = -60 * math.pi / 180
link_lengths = (14.2, 14.2, 4.5)

theta_0 = [-60*math.pi/180, -45*math.pi/180, 15*math.pi/180]  # Initial angles for three joints
theta_f = list(inverse_kinematics(x_desired, y_desired, theta_desired, *link_lengths))  # Final angles for three joints
theta_dot_0 = [0, 0, 0]
theta_dot_f = [0, 0, 0]
theta_dotdot_0 = [0, 0, 0]
theta_dotdot_f = [0, 0, 0]

# Define the time duration and create a time array
t_f = 10  # seconds
timesteps_per_second = 100
time = np.linspace(0, t_f, t_f * timesteps_per_second)

# Plot the results in a 2x3 grid
fig, axs = plt.subplots(2, 3, figsize=(18, 12))  # Adjusting the figure size for better visualization

# Coefficients for the matrix (same for all joints as t_f is the same)
a1 = t_f ** 3
b1 = t_f ** 4
c1 = t_f ** 5
a2 = 3 * (t_f ** 2)
b2 = 4 * (t_f ** 3)
c2 = 5 * (t_f ** 4)
a3 = 6 * t_f
b3 = 12 * (t_f ** 2)
c3 = 20 * (t_f ** 3)

# Matrix A is the same for all joints
A = np.array([[a1, b1, c1],
              [a2, b2, c2],
              [a3, b3, c3]])

# Arrays to store position, velocity, and acceleration for each joint
theta_all = []
theta_dot_all = []
theta_dotdot_all = []
end_effector_trajectory_x = []
end_effector_trajectory_y = []

for i in range(3):
    # Constants matrix for each joint
    d1 = theta_f[i] - theta_0[i] - theta_dot_0[i] * t_f - 0.5 * theta_dotdot_0[i] * (t_f ** 2)
    d2 = theta_dot_f[i] - theta_dot_0[i] - theta_dotdot_0[i] * t_f
    d3 = theta_dotdot_f[i] - theta_dotdot_0[i]
    B = np.array([d1, d2, d3])

    # Initial coefficients
    a_0 = theta_0[i]
    a_1 = theta_dot_0[i]
    a_2 = theta_dotdot_0[i] / 2

    # Solve for a_3, a_4, a_5
    solution = np.linalg.solve(A, B)
    a_3, a_4, a_5 = solution

    # Calculate position, velocity, and acceleration for this joint
    theta = a_0 + a_1 * time + a_2 * time ** 2 + a_3 * time ** 3 + a_4 * time ** 4 + a_5 * time ** 5
    theta_dot = a_1 + 2 * a_2 * time + 3 * a_3 * time ** 2 + 4 * a_4 * time ** 3 + 5 * a_5 * time ** 4
    theta_dotdot = 2 * a_2 + 6 * a_3 * time + 12 * a_4 * time ** 2 + 20 * a_5 * time ** 3

    # Append results to the lists
    theta_all.append(theta)
    theta_dot_all.append(theta_dot)
    theta_dotdot_all.append(theta_dotdot)

    # Calculating the end-effector position at each time step
    if i == 2:
        for t in range(len(time)):
            theta1 = theta_all[0][t]
            theta2 = theta_all[1][t]
            theta3 = theta_all[2][t]
            x_pos = link_lengths[0] * np.cos(theta1) + link_lengths[1] * np.cos(theta1 + theta2) + link_lengths[2] * np.cos(theta1 + theta2 + theta3)
            y_pos = link_lengths[0] * np.sin(theta1) + link_lengths[1] * np.sin(theta1 + theta2) + link_lengths[2] * np.sin(theta1 + theta2 + theta3)
            end_effector_trajectory_x.append(x_pos)
            end_effector_trajectory_y.append(y_pos)

# Convert lists to numpy arrays for easier manipulation
theta_all = np.array(theta_all)
theta_dot_all = np.array(theta_dot_all)
theta_dotdot_all = np.array(theta_dotdot_all)

# Plot for joint positions (row 1, column 1)
for i in range(3):
    axs[0, 0].plot(time, theta_all[i], label=f'Joint {i + 1}')

axs[0, 0].set_title('Joint Positions (theta)')
axs[0, 0].set_xlabel('Time (seconds)')
axs[0, 0].set_ylabel('Position (radians)')
axs[0, 0].legend()
axs[0, 0].grid(True)

# Plot for joint velocities (row 1, column 2)
for i in range(3):
    axs[0, 1].plot(time, theta_dot_all[i], label=f'Joint {i + 1}')

axs[0, 1].set_title('Joint Velocities (theta_dot)')
axs[0, 1].set_xlabel('Time (seconds)')
axs[0, 1].set_ylabel('Velocity (radians/second)')
axs[0, 1].legend()
axs[0, 1].grid(True)

# Plot for joint accelerations (row 1, column 3)
for i in range(3):
    axs[0, 2].plot(time, theta_dotdot_all[i], label=f'Joint {i + 1}')

axs[0, 2].set_title('Joint Accelerations (theta_dotdot)')
axs[0, 2].set_xlabel('Time (seconds)')
axs[0, 2].set_ylabel('Acceleration (radians/second^2)')
axs[0, 2].legend()
axs[0, 2].grid(True)

# Second row: Plot the initial and final manipulator configurations
# Plot the initial manipulator configuration (row 2, column 1)
plot_manipulator(theta_0, link_lengths, axs[1, 0], 'Initial Manipulator Configuration')

# Plot the final manipulator configuration (row 2, column 2)
plot_manipulator(theta_f, link_lengths, axs[1, 1], 'Final Manipulator Configuration')

# Dexterous workspace plot with manipulator
l1, l2, l3 = link_lengths
theta1_array = np.deg2rad(np.arange(-90, 91, 3))
theta2_array = np.deg2rad(np.arange(-90, 91, 3))
theta3_array = np.deg2rad(np.arange(-90, 91, 3))

# Loop to create the dexterous workspace
for theta1 in theta1_array:
    for theta2 in theta2_array:
        theta3 = theta3_array
        X = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
        Y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
        axs[1, 2].plot(X, Y, 'y.', markersize=0.25)

# Adding the manipulator in its final configuration
plot_manipulator(theta_f, link_lengths, axs[1, 2], 'Dexterous Workspace with Manipulator')

# Plot the end-effector trajectory
axs[1, 2].plot(end_effector_trajectory_x, end_effector_trajectory_y, color='red', label='End-effector Trajectory')
axs[1, 2].scatter([end_effector_trajectory_x[0], end_effector_trajectory_x[-1]],
                  [end_effector_trajectory_y[0], end_effector_trajectory_y[-1]],
                  color='red', s=50, label='Start/End Points')

axs[1, 2].legend()

# Adjust layout and show the plots
plt.tight_layout()
plt.show()
