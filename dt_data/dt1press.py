import matplotlib.pyplot as plt

# Define a list to store pressure data
pressure_hpa = []

# Read data from the text file
with open('dt1cut.txt', 'r') as file:
    for line in file:
        # Split the line by semicolon and extract values
        values = line.strip('{};').split(';')
        for value in values:
            try:
                key, val = value.split(':')
                if key == 'pressure(hPa)':
                    pressure_hpa.append(float(val))
            except ValueError:
                print(f"Issue parsing line: {line}")

# Create a list of timestamps for plotting
timestamps = list(range(len(pressure_hpa)))

# Plot pressure data
plt.figure(figsize=(10, 6))
plt.plot(timestamps, pressure_hpa, label='Pressure (hPa)')
plt.xlabel('Time (index)')
plt.ylabel('Pressure (hPa)')
plt.title('Pressure Data Over Time')
plt.legend()
plt.grid(True)
plt.show()
