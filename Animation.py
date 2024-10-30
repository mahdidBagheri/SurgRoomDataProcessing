import numpy as np
import pandas as pd
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R

# Function to convert quaternion to rotation matrix
def quaternion_to_matrix(q):
    q0, q1, q2, q3 = q
    return np.array([
        [1 - 2 * (q2 ** 2 + q3 ** 2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 ** 2 + q3 ** 2), 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 ** 2 + q2 ** 2)]
    ])

# Function to calculate Euler angles between two rotation matrices
def calculate_euler_angles(matrix1, matrix2):
    rotation_diff = matrix2 @ np.linalg.inv(matrix1)
    r = R.from_matrix(rotation_diff)
    return r.as_euler('xyz', degrees=True)

# Function to calculate the absolute difference of Euler angles
def calculate_absolute_difference(euler_angles):
    return np.sqrt(np.sum(np.square(euler_angles)))

# Read the CSV file
data = pd.read_csv('combined_data.csv')

# Extract quaternion data
hl_quaternions = data[['hl_Q0', 'hl_Qx', 'hl_Qy', 'hl_Qz']].values
ex_quaternions = data[['ex_Q0', 'ex_Qx', 'ex_Qy', 'ex_Qz']].values
timestamps = data['t'].values

# Calculate and print Euler angles and their absolute differences for all frames
for i in range(len(data)):
    hl_matrix = quaternion_to_matrix(hl_quaternions[i])
    ex_matrix = quaternion_to_matrix(ex_quaternions[i])
    euler_angles = calculate_euler_angles(hl_matrix, ex_matrix)
    abs_diff = calculate_absolute_difference(euler_angles)
    print(f"Timestamp: {timestamps[i]}, Frame {i}: Euler angles (degrees) - Roll: {euler_angles[0]:.2f}, Pitch: {euler_angles[1]:.2f}, Yaw: {euler_angles[2]:.2f}, Absolute Difference: {abs_diff:.2f}")

# Create figure and 3D axes
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Initialize the coordinate frames
hl_frame_x = ax.quiver(0, 0, 0, 1, 0, 0, color='b', label='hl')
hl_frame_y = ax.quiver(0, 0, 0, 0, 1, 0, color='b')
hl_frame_z = ax.quiver(0, 0, 0, 0, 0, 1, color='b')
ex_frame_x = ax.quiver(0, 0, 0, 1, 0, 0, color='r', label='ex')
ex_frame_y = ax.quiver(0, 0, 0, 0, 1, 0, color='r')
ex_frame_z = ax.quiver(0, 0, 0, 0, 0, 1, color='r')

# Store the quiver objects for later removal
quivers = [hl_frame_x, hl_frame_y, hl_frame_z, ex_frame_x, ex_frame_y, ex_frame_z]

# Function to update the frames
def update(frame):
    global quivers
    # Remove the previous quivers
    for quiver in quivers:
        quiver.remove()

    hl_matrix = quaternion_to_matrix(hl_quaternions[frame])
    ex_matrix = quaternion_to_matrix(ex_quaternions[frame])

    # Create new quivers for hl
    hl_frame_x = ax.quiver(0, 0, 0, hl_matrix[0, 0], hl_matrix[1, 0], hl_matrix[2, 0], color='b', label='hl')
    hl_frame_y = ax.quiver(0, 0, 0, hl_matrix[0, 1], hl_matrix[1, 1], hl_matrix[2, 1], color='b')
    hl_frame_z = ax.quiver(0, 0, 0, hl_matrix[0, 2], hl_matrix[1, 2], hl_matrix[2, 2], color='b')

    # Create new quivers for ex
    ex_frame_x = ax.quiver(0, 0, 0, ex_matrix[0, 0], ex_matrix[1, 0], ex_matrix[2, 0], color='r', label='ex')
    ex_frame_y = ax.quiver(0, 0, 0, ex_matrix[0, 1], ex_matrix[1, 1], ex_matrix[2, 1], color='r')
    ex_frame_z = ax.quiver(0, 0, 0, ex_matrix[0, 2], ex_matrix[1, 2], ex_matrix[2, 2], color='r')

    # Update the quivers list
    quivers = [hl_frame_x, hl_frame_y, hl_frame_z, ex_frame_x, ex_frame_y, ex_frame_z]

    return quivers

# Function to update the plot with slider
def update_plot(val):
    frame = int(slider.val)
    update(frame)
    fig.canvas.draw_idle()

# Create the slider
ax_slider = plt.axes([0.2, 0.01, 0.65, 0.03], facecolor='lightgoldenrodyellow')
slider = Slider(ax_slider, 'Frame', 0, len(data) - 1, valinit=0, valstep=1)
slider.on_changed(update_plot)

# Add labels, title, and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Quaternion Rotation Visualization')
ax.legend()

plt.show()
