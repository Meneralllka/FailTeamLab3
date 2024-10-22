import numpy as np
import matplotlib.pyplot as plt

# Link lengths (in cm)
l1 = 14.2
l2 = 14.2
l3 = 4.5  # Length of the third link

# Joint angle limitations (degrees)
theta1_min = -160
theta1_max = 160
theta2_min = -90
theta2_max = 90
theta3_min = -30
theta3_max = 30

# Create arrays of joint angles with one-degree increments
theta1_array = np.deg2rad(np.arange(theta1_min, theta1_max + 1, 2))
theta2_array = np.deg2rad(np.arange(theta2_min, theta2_max + 1, 2))
theta3_array = np.deg2rad(np.arange(theta3_min, theta3_max + 1, 2))

plt.figure(figsize=(10, 8))

# Loop over theta1
for theta1 in theta1_array:
    # Loop over theta2
    for theta2 in theta2_array:
        # Compute X and Y for all theta3
        theta3 = theta3_array
        X = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2) + l3 * np.cos(theta1 + theta2 + theta3)
        Y = l1 * np.sin(theta1) + l2 * np.sin(theta1 + theta2) + l3 * np.sin(theta1 + theta2 + theta3)
        plt.plot(X, Y, 'r.', markersize=0.5)

plt.xlabel('x (cm)')
plt.ylabel('y (cm)')
plt.title('Workspace of a 3R Planar Robot')
plt.axis('equal')
plt.grid(True)
plt.show()
