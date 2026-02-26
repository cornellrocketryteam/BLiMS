import matplotlib.pyplot as plt
import numpy as np
import serial
import math

plt.ion()
fig_map, ax_map = plt.subplots(figsize=(8, 8))

while True:
    ax_map.clear()

    target_lat = 42.446305
    target_lon = -76.463735

    current_lat = 42.445000
    current_lon = -76.463600

    # 0.002 is the right size
    lat_min = target_lat - 0.02
    lat_max = target_lat + 0.02
    lon_min = target_lon - 0.02
    lon_max = target_lon + 0.02

    ax_map.set_xlim(lon_min, lon_max)
    ax_map.set_ylim(lat_min, lat_max)


    # Plot current and target
    ax_map.plot(current_lon, current_lat, 'bo', label="Current")
    ax_map.plot(target_lon, target_lat, 'ro', label="Target")

    vector_len = 0.0008
    dx2 = vector_len * math.cos(math.radians(270.0))
    dy2 = vector_len * math.sin(math.radians(270.0))
    from matplotlib.patches import FancyArrow

    arrow = FancyArrow(
        current_lon, current_lat, dx2, dy2,
        width=0.000009,  # Controls shaft thickness
        head_width=vector_len * 0.1,
        head_length=vector_len * 0.15,
        color='green',
        length_includes_head=True
    )
    ax_map.add_patch(arrow)



    # Final map settings
    ax_map.set_title(f"Lat: {current_lat:.6f}, Lon: {current_lon:.6f},")
    ax_map.set_xlabel("Longitude")
    ax_map.set_ylabel("Latitude")
    ax_map.set_aspect('equal')
    ax_map.grid(True)
    ax_map.legend(loc="upper right")

    plt.pause(0.1)
