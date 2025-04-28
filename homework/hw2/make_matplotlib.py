import matplotlib.pyplot as plt
import numpy as np

# Using numpy
data = np.loadtxt("hw2.txt")
print(data)
t = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]

j1 = data[:, 4]
j2 = data[:, 5]
j3 = data[:, 6]
j4 = data[:, 7]
j5 = data[:, 8]
j6 = data[:, 9]
j7 = data[:, 10]

# # Create the plot 1
plt.plot(t, x, label = 'x')
plt.plot(t, y, label = 'y')
plt.plot(t, z, label = 'z')

# Add labels and title
plt.xlabel('time (x 0.1s)')
plt.ylabel('value of end effector position in frame 0 (m)')
plt.legend()
plt.title('time vs end effector position with control law 4iv and circular trajectory, kv=72, kvj=4')

#------
#plot 2
# plt.plot(t, j1, label = 'j1')
# plt.plot(t, j2, label = 'j2')
# plt.plot(t, j3, label = 'j3')
# plt.plot(t, j4, label = 'j4')
# plt.plot(t, j5, label = 'j5')
# plt.plot(t, j6, label = 'j6')
# plt.plot(t, j7, label = 'j7')

# plt.xlabel('time (x 0.1s)')
# plt.ylabel('joint position (rad)')
# plt.legend()
# plt.title('time vs joint position with control law 4iv and circular trajectory, kv=72, kvj=4')


# Show the plot
plt.show()