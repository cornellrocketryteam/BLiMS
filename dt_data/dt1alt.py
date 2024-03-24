import matplotlib.pyplot as plt

# Define a list to store altitude data
altitude_m = []

# Read data from the text file
with open('dt1cut.txt', 'r') as file:
    for line in file:
        # Split the line by semicolon and extract values
        values = line.strip('{};').split(';')
        for value in values:
            try:
                key, val = value.split(':')
                if key == 'altitude(m)':
                    altitude_m.append(float(val))
            except ValueError:
                print(f"Issue parsing line: {line}")

# Create a list of timestamps for plotting
timestamps = list(range(len(altitude_m)))

# Plot altitude data
plt.figure(figsize=(10, 6))
plt.plot(timestamps, altitude_m, label='Altitude (m)')
plt.xlabel('Time (index)')
plt.ylabel('Altitude (m)')
plt.title('Altitude Data Over Time')
plt.legend()
plt.grid(True)
plt.show()
