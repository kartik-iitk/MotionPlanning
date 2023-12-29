import numpy as np
from scipy.interpolate import CubicSpline

# Read waypoints from the "output.txt" file
file_path = "output.txt"
waypoints = np.loadtxt(file_path)

# Separate x and y values
x_values, y_values = waypoints.T

# Generate a smooth trajectory using Cubic Spline
t = np.arange(len(waypoints))
cs_x = CubicSpline(t, x_values, bc_type='clamped')
cs_y = CubicSpline(t, y_values, bc_type='clamped')

# Array to store robot positions
robot_positions = []

# Calculate robot positions without running the animation
for frame in np.linspace(0, len(waypoints) - 1, 100):
    x_robot = cs_x(frame)
    y_robot = cs_y(frame)
    robot_positions.append((x_robot, y_robot))

# Display the array of robot positions
print("Robot Positions:", robot_positions)
output_file_path = "output1.txt"

# Write the array to the text file
np.savetxt(output_file_path, robot_positions, fmt='%.8f', delimiter=' ')

print(f"The array has been written to {output_file_path}")

# Plot all the points of robot_positions
# robot_positions_array = np.array(robot_positions).T
# plt.plot(robot_positions_array[0], robot_positions_array[1], 'o', label='Robot Positions')
# plt.title('Robot Positions Without Animation')
# plt.xlabel('X-axis')
# plt.ylabel('Y-axis')
# plt.legend()
# plt.show()
