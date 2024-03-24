import matplotlib.pyplot as plt
import numpy as np

def convert_successive_angles(initial_angles, successive_angles):
    """
    Convert successive Euler angles to relative angles from an initial angle.

    Args:
    initial_angles (tuple or list): Initial Euler angles (X, Y, Z) in degrees.
    successive_angles (list of tuples or lists): List of successive Euler angles (X, Y, Z) in degrees.

    Returns:
    list of tuples: List of relative Euler angles (X_rel, Y_rel, Z_rel) in degrees.
    """
    # Convert initial angles to radians for calculations
    initial_rad = [angle * (np.pi / 180) for angle in initial_angles]

    # Initialize list to store relative angles
    relative_angles = []

    # Calculate relative angles for each successive angle
    for angles in successive_angles:
        # Convert angles to radians for calculations
        angles_rad = [angle * (np.pi / 180) for angle in angles]

        # Calculate relative angles
        rel_angles = [angles_rad[i] - initial_rad[i] for i in range(3)]

        # Convert relative angles back to degrees and append to the result list
        rel_degrees = [angle * (180 / np.pi) for angle in rel_angles]
        relative_angles.append(tuple(rel_degrees))

        # Update initial angles for the next iteration
        initial_rad = angles_rad

    return relative_angles

# Define lists to store IMU data
imu_x = []
imu_y = []
imu_z = []

# Read data from the text file
with open('dt1cut.txt', 'r') as file:
    for line in file:
        # Split the line by semicolon and extract values
        values = line.strip('{};').split(';')
        for value in values:
            try:
                key, val = value.split(':')
                if key == 'IMU(deg) X':
                    imu_x.append(float(val))
                elif key == 'Y':
                    imu_y.append(float(val))
                elif key == 'Z':
                    imu_z.append(float(val))
            except ValueError:
                print(f"Issue parsing line: {line}")

# Convert IMU angles to relative angles
initial_angles = (0, 0, 0)  # Initial Euler angles (X, Y, Z) in degrees
successive_angles = list(zip(imu_x, imu_y, imu_z))  # Successive Euler angles in degrees
relative_angles = convert_successive_angles(initial_angles, successive_angles)

# Create a list of timestamps for plotting
timestamps = list(range(len(relative_angles)))  # Assuming relative_angles has the same length as imu_x, imu_y, imu_z

# Plot relative Euler angles from the 800th time index onwards
start_index = 950
timestamps = timestamps[start_index:]
relative_angles = relative_angles[start_index:]

# Plot relative Euler angles
plt.figure(figsize=(10, 6))
plt.plot(timestamps, [angle[0] for angle in relative_angles], label='Relative Euler X')
plt.plot(timestamps, [angle[1] for angle in relative_angles], label='Relative Euler Y')
plt.plot(timestamps, [angle[2] for angle in relative_angles], label='Relative Euler Z')
plt.xlabel('Time (index)')
plt.ylabel('Relative Angles (degrees)')
plt.title('Relative Euler Angles Over Time (Starting from Index 950)')
plt.legend()
plt.grid(True)
plt.show()
