import matplotlib.pyplot as plt
import numpy as np
import serial
import math
from collections import deque
from matplotlib.patches import FancyArrow

# === CONFIGURATION ===
serial_port = 'COM7'
baud_rate = 115200
window_seconds = 30
max_rate_hz = 10
max_len = window_seconds * max_rate_hz

# === SERIAL ===
try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
except serial.SerialException as e:
    exit(f"Could not open serial port {serial_port}: {e}")

# === PLOT SETUP ===
plt.ion()
fig, (ax_map, ax_ctrl) = plt.subplots(2, 1, figsize=(8, 10), gridspec_kw={'height_ratios': [3, 1]})
fig.tight_layout(pad=3.0)

# === DATA STORAGE ===
gps_trail = deque(maxlen=max_len)        # (lat, lon, timestamp)
timestamps = deque(maxlen=max_len)       # in seconds
p_terms = deque(maxlen=max_len)
i_terms = deque(maxlen=max_len)

# === MAIN LOOP ===
while True:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue

        parts = line.split(',')
        if len(parts) != 10:
            continue

        try:
            current_lat, current_lon, target_lat, target_lon = map(float, parts[0:4])
            heading, target_heading = map(float, parts[4:6])
            motor_position = float(parts[6])
            timestamp_ms = float(parts[7])
            P_term = float(parts[8])
            I_term = float(parts[9])
        except ValueError:
            continue

        error = target_heading - heading
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        timestamp = timestamp_ms / 1000.0
        gps_trail.append((current_lat, current_lon, timestamp))
        timestamps.append(timestamp)
        p_terms.append(P_term)
        i_terms.append(I_term)

        # === MAP PLOT ===
        ax_map.clear()

        lat_min = target_lat - 0.002
        lat_max = target_lat + 0.002
        lon_min = target_lon - 0.002
        lon_max = target_lon + 0.002

        ax_map.set_xlim(lon_min, lon_max)
        ax_map.set_ylim(lat_min, lat_max)

        # Trail
        recent_trail = [(lat, lon) for (lat, lon, t) in gps_trail if timestamp - t <= window_seconds]
        if len(recent_trail) > 1:
            trail_lats, trail_lons = zip(*recent_trail)
            ax_map.plot(trail_lons, trail_lats, color='gray', linewidth=1, alpha=0.7, label="GPS Trail")
        else:
            trail_lats, trail_lons = [current_lat], [current_lon]

        ax_map.plot(current_lon, current_lat, 'bo', label="Current")
        ax_map.plot(target_lon, target_lat, 'ro', label="Target")

        # Heading vector
        heading_rad = math.radians(heading)
        dx1 = 0.0008 * math.cos(heading_rad)
        dy1 = 0.0008 * math.sin(heading_rad)
        arrow = FancyArrow(current_lon, current_lat, dx1, dy1,
                           width=0.000009, head_width=0.00008, head_length=0.00012,
                           color='blue', length_includes_head=True)
        ax_map.add_patch(arrow)

        # Target bearing vector
        bearing_rad = math.radians(target_heading)
        dx2 = 0.0008 * math.cos(bearing_rad)
        dy2 = 0.0008 * math.sin(bearing_rad)
        arrow2 = FancyArrow(current_lon, current_lat, dx2, dy2,
                            width=0.000009, head_width=0.00008, head_length=0.00012,
                            color='green', length_includes_head=True)
        ax_map.add_patch(arrow2)

        ax_map.set_title(f"Lat: {current_lat:.6f}, Lon: {current_lon:.6f}, Head: {heading:.1f}°, Target Head: {target_heading:.1f}°, Motor Input: {motor_position:.1f}")
        ax_map.set_xlabel("Longitude")
        ax_map.set_ylabel("Latitude")
        ax_map.set_aspect('equal')
        ax_map.grid(True)
        ax_map.legend(loc="upper right")

        # === CONTROL PLOT ===
        ax_ctrl.clear()
        recent_times = [t for t in timestamps if timestamp - t <= window_seconds]
        t0 = recent_times[0] if recent_times else timestamp
        rel_times = [t - t0 for t in timestamps][-len(p_terms):]

        ax_ctrl.plot(rel_times, p_terms, label='P Term', color='orange')
        ax_ctrl.plot(rel_times, i_terms, label='I Term', color='purple')
        ax_ctrl.set_ylim(-1, 1)
        ax_ctrl.set_xlabel("Time (s)")
        ax_ctrl.set_ylabel("PI Terms")
        ax_ctrl.grid(True)
        ax_ctrl.legend(loc="upper right")

        plt.pause(0.05)

    except Exception:
        continue
