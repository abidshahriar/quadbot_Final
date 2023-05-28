import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create a figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Define the cuboid parameters
center = (0.5, 0.5, 0.5)
length = 0.5
width = 0.3
height = 0.2

# Create coordinates of cuboid
x = [center[0] - length / 2, center[0] + length / 2, center[0] + length / 2, center[0] - length / 2,
     center[0] - length / 2, center[0] - length / 2, center[0] + length / 2, center[0] + length / 2]
y = [center[1] - width / 2, center[1] - width / 2, center[1] + width / 2, center[1] + width / 2,
     center[1] - width / 2, center[1] + width / 2, center[1] + width / 2, center[1] - width / 2]
z = [center[2] - height / 2, center[2] - height / 2, center[2] - height / 2, center[2] - height / 2,
     center[2] - height / 2, center[2] + height / 2, center[2] + height / 2, center[2] + height / 2]

# Create arrays for surface coordinates
X, Y = np.meshgrid(x, y)
Z = np.full_like(X, center[2] - height / 2)

# Plot the bottom surface
ax.plot_surface(X, Y, Z, alpha=0.5)

# Plot the top surface
Z = np.full_like(X, center[2] + height / 2)
ax.plot_surface(X, Y, Z, alpha=0.5)

# Plot the four side surfaces
Z = np.linspace(center[2] - height / 2, center[2] + height / 2, 2)
X, Z = np.meshgrid(x, Z)
ax.plot_surface(X, y[0], Z, alpha=0.5)
ax.plot_surface(X, y[2], Z, alpha=0.5)
Y, Z = np.meshgrid(y, Z)
ax.plot_surface(x[0], Y, Z, alpha=0.5)
ax.plot_surface(x[1], Y, Z, alpha=0.5)

# Set labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Small Solid 3D Cuboid')

# Show the plot
plt.show()
