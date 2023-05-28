import matplotlib.pyplot as plt
import numpy as np

# Generate some random data
x = np.random.randn(100)
y = np.random.randn(100)

# Create a scatter plot with a different color border
plt.scatter(x, y, c='blue', edgecolor='red')

# Show the plot
plt.show()
