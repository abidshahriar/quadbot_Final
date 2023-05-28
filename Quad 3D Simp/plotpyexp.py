import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3D

# Create a figure and a 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Generate some random data
p = np.random.rand(3, 10)  # Example coordinates

# Example line widths and edge colors
line_widths = [5.0, 3.0, 4.5]
edge_colors = ['red', 'green', 'blue']

# Iterate over line widths and edge colors
for i, (lw, ec) in enumerate(zip(line_widths, edge_colors)):
    x = np.asarray(p[0, :]).flatten()
    y = np.asarray(p[1, :]).flatten()
    z = np.asarray(p[2, :]).flatten()

    # Create a Line3D collection with individual colors
    segments = [list(zip(x, y, z))]
    lc = Line3D(*segments, color='black', linewidth=lw)
    lc.set_edgecolor(ec)
    ax.add_collection3d(lc)

# Show the plot
plt.show()
