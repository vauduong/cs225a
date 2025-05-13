import matplotlib.pyplot as plt
import numpy as np

# Using numpy
data = np.loadtxt("hw3.txt")
print(data)
t = data[:, 0]
x = data[:, 1]
y = data[:, 2]
z = data[:, 3]

x_desired = data[:, 4]
y_desired = data[:, 5]
z_desired = data[:, 6]

x_dot_1 = data[:, 7]
x_dot_2 = data[:, 8]
x_dot_3 = data[:, 9]
vmax = data[:, 10]

# # # Create the plot 1
# plt.plot(t, x, label = 'x')
# plt.plot(t, x_desired, label = 'x_desired')
# plt.plot(t, y, label = 'y')
# plt.plot(t, y_desired, label = 'y_desired')
# plt.plot(t, z, label = 'z')
# plt.plot(t, z_desired, label = 'z_desired')

# # Add labels and title
# plt.xlabel('time (x 0.1s)')
# plt.ylabel('value of end effector position in frame 0 (m)')
# plt.legend()
# plt.title('time vs end effector position and desired position, control law 4b')

#plot dphi
plt.plot(t, x_dot_1, label = 'x_dot_1')
plt.plot(t, x_dot_2, label = 'x_dot_2')
plt.plot(t, x_dot_3, label = 'x_dot_3')
plt.plot(t, vmax, label = 'vmax')
plt.xlabel('time (x 0.1s)')
plt.ylabel('value of x_dot (m/s)')
plt.legend()
plt.title('time vs x_dot, control law 4b')


# #------
# # #plot 2
# plt.plot(t, q_4lower, label = 'joint 4 lower limit')
# plt.plot(t, q_4, label = 'joint 4 position')
# plt.plot(t, q_4upper, label = 'joint 4 upper limit')
# plt.plot(t, q_6lower, label = 'joint 6 lower limit')
# plt.plot(t,q_6, label = 'joint 6 position')
# plt.plot(t, q_6upper, label = 'joint 6 upper limit')

# plt.xlabel('time (x 0.1s)')
# plt.ylabel('joint position (rad)')
# plt.legend()
# plt.title('time vs joint position and desired joint position for joints 4 and 6, control law 3')


# Show the plot
plt.show()