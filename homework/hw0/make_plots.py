import matplotlib.pyplot as plt
import numpy as np

# Using numpy
data = np.loadtxt("q2-f-i.txt")
print(data)
t = np.arange(-90.0, 90.0, 180.0/250.0)
g0 = data[:, 0]
g1 = data[:, 1]
g2 = data[:, 2]

# Create the plot
plt.plot(t, g0, label = 'g0')
plt.plot(t, g1, label = 'g1')
plt.plot(t, g2, label = 'g2')

# Add labels and title
plt.xticks([-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90])
plt.xlabel('theta3')
plt.ylabel('gravity values of each joint')
plt.legend()
plt.title('q2-f-i')

# Show the plot
plt.show()