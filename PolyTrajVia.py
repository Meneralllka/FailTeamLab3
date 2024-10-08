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
link_lengths = (14.2, 14.2, 4.5)

# Initial joint angles (from your original code)
theta_0 = [0 * np.pi / 180, 0 * np.pi / 180, 0 * np.pi / 180]  # Initial joint angles in radians

# Compute initial end-effector position using forward kinematics
def forward_kinematics(theta_angles, link_lengths):
    theta1, theta2, theta3 = theta_angles
    l1, l2, l3 = link_lengths
    x = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
    y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
    return x, y

x_initial, y_initial = forward_kinematics(theta_0, link_lengths)
theta_initial = theta_0[0] + theta_0[1] + theta_0[2]  # Sum of initial joint angles

# Desired end-effector position and orientation
x_desired = 0
y_desired = 30
theta_desired = 90 * np.pi / 180  # Desired orientation in radians

# Compute final joint angles using inverse kinematics
theta_f = list(inverse_kinematics(x_desired, y_desired, theta_desired, *link_lengths))  # Final joint angles

# Number of via points (you can change this value)
num_via_points = 4  # Number of via points along the line

# Generate via points along the straight line between initial and desired positions
total_points = num_via_points + 2  # Including initial and final points
x_points = np.linspace(x_initial, x_desired, total_points)
y_points = np.linspace(y_initial, y_desired, total_points)
theta_points = np.linspace(theta_initial, theta_desired, total_points)  # Linear interpolation of orientation

# Compute joint angles for each via point using inverse kinematics
all_points = []
for i in range(total_points):
    x = x_points[i]
    y = y_points[i]
    theta = theta_points[i]
    try:
        # Inverse kinematics function returns multiple solutions; select one
        joint_angles = inverse_kinematics(x, y, theta, *link_lengths)
        all_points.append(joint_angles)
    except Exception as e:
        print(f"Inverse kinematics failed at point ({x}, {y}): {e}")
        exit()

# Define time durations for each segment
total_time = 10  # Total time for the entire trajectory in seconds
num_segments = len(all_points) - 1
time_per_segment = total_time / num_segments

# Time array for the entire trajectory
timesteps_per_second = 100
time = np.linspace(0, total_time, int(total_time * timesteps_per_second))

# Arrays to store position, velocity, and acceleration for each joint over the entire trajectory
theta_all = np.zeros((3, len(time)))
theta_dot_all = np.zeros((3, len(time)))
theta_dotdot_all = np.zeros((3, len(time)))

# Arrays to store the end-effector trajectory
end_effector_trajectory_x = []
end_effector_trajectory_y = []

# Time array for each segment
time_segment = np.linspace(0, time_per_segment, int(time_per_segment * timesteps_per_second))

# Initialize the starting index for the overall time array
start_idx = 0

# Loop over each segment to compute the trajectory
for seg in range(num_segments):
    theta_start = all_points[seg]
    theta_end = all_points[seg + 1]

    # Boundary conditions for the current segment
    theta_dot_start = [0, 0, 0] if seg == 0 else theta_dot_end.tolist()  # Continuity of velocity
    theta_dot_end = [0, 0, 0] if seg == num_segments - 1 else [0, 0, 0]  # Zero velocity at via points

    theta_dotdot_start = [0, 0, 0] if seg == 0 else theta_dotdot_end.tolist()  # Continuity of acceleration
    theta_dotdot_end = [0, 0, 0] if seg == num_segments - 1 else [0, 0, 0]  # Zero acceleration at via points

    # Coefficients for the matrix (same for all joints as time_per_segment is the same)
    t_f = time_per_segment
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

    # Number of time steps in this segment
    num_steps = len(time_segment)
    end_idx = start_idx + num_steps

    # Loop over each joint
    for i in range(3):
        # Constants matrix for each joint
        d1 = theta_end[i] - theta_start[i] - theta_dot_start[i] * t_f - 0.5 * theta_dotdot_start[i] * (t_f ** 2)
        d2 = theta_dot_end[i] - theta_dot_start[i] - theta_dotdot_start[i] * t_f
        d3 = theta_dotdot_end[i] - theta_dotdot_start[i]
        B = np.array([d1, d2, d3])

        # Initial coefficients
        a_0 = theta_start[i]
        a_1 = theta_dot_start[i]
        a_2 = theta_dotdot_start[i] / 2

        # Solve for a_3, a_4, a_5
        solution = np.linalg.solve(A, B)
        a_3, a_4, a_5 = solution

        # Calculate position, velocity, and acceleration for this joint over the segment
        t = time_segment
        theta = a_0 + a_1 * t + a_2 * t ** 2 + a_3 * t ** 3 + a_4 * t ** 4 + a_5 * t ** 5
        theta_dot = a_1 + 2 * a_2 * t + 3 * a_3 * t ** 2 + 4 * a_4 * t ** 3 + 5 * a_5 * t ** 4
        theta_dotdot = 2 * a_2 + 6 * a_3 * t + 12 * a_4 * t ** 2 + 20 * a_5 * t ** 3

        # Store the computed values in the overall arrays
        theta_all[i, start_idx:end_idx] = theta
        theta_dot_all[i, start_idx:end_idx] = theta_dot
        theta_dotdot_all[i, start_idx:end_idx] = theta_dotdot

    # Update theta_dot_end and theta_dotdot_end for continuity in the next segment
    theta_dot_end = theta_dot_all[:, end_idx - 1]
    theta_dotdot_end = theta_dotdot_all[:, end_idx - 1]

    # Update the starting index for the next segment
    start_idx = end_idx

# Calculating the end-effector position at each time step
for t_idx in range(len(time)):
    theta1 = theta_all[0][t_idx]
    theta2 = theta_all[1][t_idx]
    theta3 = theta_all[2][t_idx]
    x_pos = link_lengths[0] * np.cos(theta1) + link_lengths[1] * np.cos(theta1 + theta2) + link_lengths[2] * np.cos(theta1 + theta2 + theta3)
    y_pos = link_lengths[0] * np.sin(theta1) + link_lengths[1] * np.sin(theta1 + theta2) + link_lengths[2] * np.sin(theta1 + theta2 + theta3)
    end_effector_trajectory_x.append(x_pos)
    end_effector_trajectory_y.append(y_pos)

# Plot the results in a 2x3 grid
fig, axs = plt.subplots(2, 3, figsize=(18, 12))  # Adjusting the figure size for better visualization

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
theta1_array = np.deg2rad(np.arange(-90, 91, 5))
theta2_array = np.deg2rad(np.arange(-90, 91, 5))
theta3_array = np.deg2rad(np.arange(-90, 91, 5))

# Create the dexterous workspace
X_ws = []
Y_ws = []
for theta1 in theta1_array:
    for theta2 in theta2_array:
        for theta3 in theta3_array:
            X = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
            Y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
            X_ws.append(X)
            Y_ws.append(Y)
axs[1, 2].scatter(X_ws, Y_ws, c='y', s=0.1)

# Adding the manipulator in its final configuration
plot_manipulator(theta_f, link_lengths, axs[1, 2], 'Dexterous Workspace with Manipulator')

# Plot the end-effector trajectory
axs[1, 2].plot(end_effector_trajectory_x, end_effector_trajectory_y, color='red', label='End-effector Trajectory')
axs[1, 2].scatter([end_effector_trajectory_x[0], end_effector_trajectory_x[-1]],
                  [end_effector_trajectory_y[0], end_effector_trajectory_y[-1]],
                  color='red', s=50, label='Start/End Points')

# Plot via points in the workspace
for i in range(1, len(all_points) - 1):
    x_via, y_via = forward_kinematics(all_points[i], link_lengths)
    axs[1, 2].scatter(x_via, y_via, color='blue', s=50, label='Via Point')

axs[1, 2].legend()

# Adjust layout and show the plots
plt.tight_layout()
plt.show()
