"""
car_test_visualizer.py — Real-time visualizer for blims_car_test.cpp

Reads the 13-field CSV from serial and displays:
  1. Map: GPS trail, position, heading arrow, bearing arrow, target
  2. Altitude + phase timeline
  3. PI controller terms + motor position

CSV format (13 fields):
  lat,lon,target_lat,target_lon,heading,bearing,motor_pos,timestamp_ms,P,I,phase,altitude,loiter_step

Heading/bearing are in geographic convention (0=North, CW).

Usage:
  python car_test_visualizer.py
  python car_test_visualizer.py COM7          # specify port
  python car_test_visualizer.py /dev/ttyACM0  # Linux
"""

import sys
import math
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from collections import deque

# =============================================================================
# CONFIGURATION
# =============================================================================

BAUD_RATE = 115200
WINDOW_SECONDS = 60        # how many seconds of trail/history to show
MAX_RATE_HZ = 20
MAX_LEN = WINDOW_SECONDS * MAX_RATE_HZ

# Map zoom (degrees of lat/lon padding around target)
MAP_PADDING = 0.002

# Phase display info
PHASE_NAMES = ["HELD", "TRACK", "DOWNWIND", "BASE", "FINAL", "NEUTRAL", "LOITER"]
PHASE_COLORS = {
    0: "#888888",   # HELD     - gray
    1: "#2196F3",   # TRACK    - blue
    2: "#FF9800",   # DOWNWIND - orange
    3: "#9C27B0",   # BASE     - purple
    4: "#F44336",   # FINAL    - red
    5: "#424242",   # NEUTRAL  - dark gray
    6: "#00BCD4",   # LOITER   - cyan
}

LOITER_NAMES = ["TURN_R", "PAUSE_R", "TURN_L", "PAUSE_L"]


# =============================================================================
# SERIAL PORT SELECTION
# =============================================================================

def find_serial_port():
    """Auto-detect or use command line arg."""
    if len(sys.argv) > 1:
        return sys.argv[1]

    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found. Specify one as argument.")
        sys.exit(1)

    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")

    if len(ports) == 1:
        print(f"Auto-selecting: {ports[0].device}")
        return ports[0].device

    choice = input("Select port number: ").strip()
    return ports[int(choice)].device


# =============================================================================
# DATA STORAGE
# =============================================================================

timestamps    = deque(maxlen=MAX_LEN)   # seconds since boot
latitudes     = deque(maxlen=MAX_LEN)
longitudes    = deque(maxlen=MAX_LEN)
headings      = deque(maxlen=MAX_LEN)   # geographic
bearings      = deque(maxlen=MAX_LEN)   # geographic
motor_positions = deque(maxlen=MAX_LEN)
p_terms       = deque(maxlen=MAX_LEN)
i_terms       = deque(maxlen=MAX_LEN)
phases        = deque(maxlen=MAX_LEN)
altitudes     = deque(maxlen=MAX_LEN)
loiter_steps  = deque(maxlen=MAX_LEN)


# =============================================================================
# HELPER: geographic heading → arrow dx/dy on lon/lat axes
# =============================================================================

def geo_heading_to_arrow(heading_deg, length):
    """Convert geographic heading (0=North, CW) to dx_lon, dy_lat."""
    rad = math.radians(heading_deg)
    dx = length * math.sin(rad)   # East component → longitude axis
    dy = length * math.cos(rad)   # North component → latitude axis
    return dx, dy


# =============================================================================
# MAIN
# =============================================================================

def main():
    port = find_serial_port()
    print(f"Opening {port} at {BAUD_RATE} baud...")

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}")

    print("Connected. Waiting for data...\n")

    # ---- Plot setup ----
    plt.ion()
    fig, (ax_map, ax_alt, ax_ctrl) = plt.subplots(
        3, 1, figsize=(9, 12),
        gridspec_kw={"height_ratios": [3, 1.2, 1]}
    )
    fig.tight_layout(pad=2.5)
    fig.canvas.manager.set_window_title("BLiMS Car Test Visualizer")

    target_lat = None
    target_lon = None

    # ---- Main loop ----
    while True:
        try:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            # Skip comment/status lines
            if raw.startswith("#"):
                print(raw)
                continue

            parts = raw.split(",")
            if len(parts) != 13:
                continue

            # Parse
            try:
                lat          = float(parts[0])
                lon          = float(parts[1])
                tgt_lat      = float(parts[2])
                tgt_lon      = float(parts[3])
                heading      = float(parts[4])
                bearing_val  = float(parts[5])
                motor_pos    = float(parts[6])
                ts_ms        = float(parts[7])
                p_term       = float(parts[8])
                i_term       = float(parts[9])
                phase        = int(parts[10])
                altitude     = float(parts[11])
                loiter_step  = int(parts[12])
            except ValueError:
                continue

            target_lat = tgt_lat
            target_lon = tgt_lon

            ts = ts_ms / 1000.0
            timestamps.append(ts)
            latitudes.append(lat)
            longitudes.append(lon)
            headings.append(heading)
            bearings.append(bearing_val)
            motor_positions.append(motor_pos)
            p_terms.append(p_term)
            i_terms.append(i_term)
            phases.append(phase)
            altitudes.append(altitude)
            loiter_steps.append(loiter_step)

            # =================================================================
            # MAP PLOT
            # =================================================================
            ax_map.clear()

            # Bounds centered on target
            ax_map.set_xlim(target_lon - MAP_PADDING, target_lon + MAP_PADDING)
            ax_map.set_ylim(target_lat - MAP_PADDING, target_lat + MAP_PADDING)

            # GPS trail (colored by phase)
            if len(latitudes) > 1:
                for i in range(1, len(latitudes)):
                    if ts - timestamps[i] > WINDOW_SECONDS:
                        continue
                    ph = phases[i] if i < len(phases) else 0
                    color = PHASE_COLORS.get(ph, "#888888")
                    ax_map.plot(
                        [longitudes[i-1], longitudes[i]],
                        [latitudes[i-1], latitudes[i]],
                        color=color, linewidth=1.5, alpha=0.7
                    )

            # Target
            ax_map.plot(target_lon, target_lat, "r^", markersize=12,
                        label="Target", zorder=5)

            # Current position
            ax_map.plot(lon, lat, "ko", markersize=7, zorder=6)

            # Heading arrow (blue) — geographic convention
            arrow_len = MAP_PADDING * 0.4
            dx_h, dy_h = geo_heading_to_arrow(heading, arrow_len)
            arrow_head = FancyArrow(
                lon, lat, dx_h, dy_h,
                width=MAP_PADDING * 0.005,
                head_width=arrow_len * 0.12,
                head_length=arrow_len * 0.15,
                color="blue", alpha=0.9,
                length_includes_head=True, zorder=7
            )
            ax_map.add_patch(arrow_head)

            # Bearing arrow (green) — geographic convention
            dx_b, dy_b = geo_heading_to_arrow(bearing_val, arrow_len)
            arrow_bear = FancyArrow(
                lon, lat, dx_b, dy_b,
                width=MAP_PADDING * 0.005,
                head_width=arrow_len * 0.12,
                head_length=arrow_len * 0.15,
                color="green", alpha=0.7,
                length_includes_head=True, zorder=7
            )
            ax_map.add_patch(arrow_bear)

            # Phase name and info
            phase_name = PHASE_NAMES[phase] if 0 <= phase <= 6 else "???"
            phase_color = PHASE_COLORS.get(phase, "#888888")
            loiter_info = ""
            if phase == 6:  # LOITER
                ln = LOITER_NAMES[loiter_step] if 0 <= loiter_step <= 3 else "?"
                loiter_info = f" [{ln}]"

            ax_map.set_title(
                f"Phase: {phase_name}{loiter_info}  |  "
                f"Alt: {altitude:.0f} ft  |  "
                f"Motor: {motor_pos:.3f}  |  "
                f"Head: {heading:.0f}°  Bear: {bearing_val:.0f}°",
                fontsize=10, fontweight="bold",
                color=phase_color
            )
            ax_map.set_xlabel("Longitude")
            ax_map.set_ylabel("Latitude")
            ax_map.set_aspect("equal")
            ax_map.grid(True, alpha=0.3)

            # Legend entries
            ax_map.plot([], [], color="blue", linewidth=2, label="Heading")
            ax_map.plot([], [], color="green", linewidth=2, label="Bearing")
            ax_map.legend(loc="upper right", fontsize=8)

            # =================================================================
            # ALTITUDE + PHASE PLOT
            # =================================================================
            ax_alt.clear()

            if len(timestamps) > 1:
                t0 = timestamps[0]
                rel_t = [t - t0 for t in timestamps]

                # Shade background by phase
                for i in range(1, len(rel_t)):
                    ph = phases[i] if i < len(phases) else 0
                    color = PHASE_COLORS.get(ph, "#888888")
                    ax_alt.axvspan(rel_t[i-1], rel_t[i],
                                   color=color, alpha=0.15)

                # Altitude line
                ax_alt.plot(rel_t, list(altitudes), color="black",
                            linewidth=1.5, label="Altitude (ft)")

                # Threshold lines
                for thresh, name in [(1000, "DOWNWIND"), (600, "BASE"),
                                     (300, "FINAL"), (100, "NEUTRAL")]:
                    ax_alt.axhline(thresh, color="gray", linestyle="--",
                                   linewidth=0.7, alpha=0.5)
                    ax_alt.text(rel_t[-1] + 0.5, thresh + 10, name,
                                fontsize=7, color="gray", va="bottom")

            ax_alt.set_ylabel("Altitude (ft)")
            ax_alt.set_xlabel("")
            ax_alt.grid(True, alpha=0.3)
            ax_alt.set_ylim(bottom=-20)

            # =================================================================
            # CONTROL PLOT
            # =================================================================
            ax_ctrl.clear()

            if len(timestamps) > 1:
                t0 = timestamps[0]
                rel_t = [t - t0 for t in timestamps]

                ax_ctrl.plot(rel_t, list(p_terms), label="P",
                             color="orange", linewidth=1)
                ax_ctrl.plot(rel_t, list(i_terms), label="I",
                             color="purple", linewidth=1)
                ax_ctrl.plot(rel_t, list(motor_positions), label="Motor",
                             color="black", linewidth=1.2, linestyle="--")

                # Neutral reference line
                ax_ctrl.axhline(0.5, color="gray", linestyle=":",
                                linewidth=0.7, alpha=0.5)

            ax_ctrl.set_ylim(-0.3, 1.0)
            ax_ctrl.set_xlabel("Time (s)")
            ax_ctrl.set_ylabel("Control")
            ax_ctrl.grid(True, alpha=0.3)
            ax_ctrl.legend(loc="upper right", fontsize=8)

            plt.pause(0.04)

        except KeyboardInterrupt:
            print("\nStopping.")
            break
        except Exception as e:
            # Silently skip bad lines, but print unexpected errors
            if "Serial" in str(type(e).__name__):
                print(f"Serial error: {e}")
                break
            continue

    ser.close()
    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()
