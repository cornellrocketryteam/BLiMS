#!/usr/bin/env python3
"""
car_test_visualizer_qt.py — Real-time BLiMS visualizer (PyQtGraph, lag-free)

Replaces the matplotlib version.  The key architectural difference:

  SerialReader (daemon thread)
      reads lines from serial port
      parses 13-field CSV
      puts parsed dicts into a thread-safe queue.Queue

  Visualizer (Qt main thread)
      QTimer fires every 50 ms
      _update() drains the ENTIRE queue each tick   ← never falls behind
      calls setData() on existing PlotDataItems      ← no clear/redraw cost

Install:
    pip install pyqtgraph PyQt6 pyserial numpy

Usage:
    python car_test_visualizer_qt.py
    python car_test_visualizer_qt.py COM7
    python car_test_visualizer_qt.py /dev/ttyACM0
"""

import sys
import math
import queue
import threading

import numpy as np
import serial
import serial.tools.list_ports

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

# ─────────────────────────────────────────────────────────────────────────────
# CONFIGURATION  (mirror values from blims_car_test.cpp / original visualizer)
# ─────────────────────────────────────────────────────────────────────────────

BAUD_RATE      = 115200
WINDOW_SECONDS = 60
MAX_RATE_HZ    = 20
MAX_LEN        = WINDOW_SECONDS * MAX_RATE_HZ   # 1 200 samples
MAP_PADDING    = 0.002                           # degrees around target
GUI_HZ         = 20                             # Qt redraw rate (timer ms = 1000/GUI_HZ)

PHASE_NAMES = ["HELD", "TRACK", "DOWNWIND", "BASE", "FINAL", "NEUTRAL", "LOITER"]
LOITER_NAMES = ["TURN_R", "PAUSE_R", "TURN_L", "PAUSE_L"]

# (R, G, B) tuples for each phase id
PHASE_RGB = {
    0: (136, 136, 136),   # HELD     – gray
    1: ( 33, 150, 243),   # TRACK    – blue
    2: (255, 152,   0),   # DOWNWIND – orange
    3: (156,  39, 176),   # BASE     – purple
    4: (244,  67,  54),   # FINAL    – red
    5: ( 66,  66,  66),   # NEUTRAL  – dark gray
    6: (  0, 188, 212),   # LOITER   – cyan
}

# Pre-build QBrush / QPen objects — avoids per-frame allocations
PHASE_BRUSH = {ph: pg.mkBrush(*rgb) for ph, rgb in PHASE_RGB.items()}
PHASE_PEN   = {ph: pg.mkPen(color=rgb, width=2)  for ph, rgb in PHASE_RGB.items()}

# Altitude phase transition thresholds (ft) — for reference lines
ALT_THRESHOLDS = {
    "DOWNWIND": 1000,
    "BASE":      600,
    "FINAL":     300,
    "NEUTRAL":   100,
}

# ─────────────────────────────────────────────────────────────────────────────
# SERIAL PORT SELECTION
# ─────────────────────────────────────────────────────────────────────────────

def find_port() -> str:
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        sys.exit("No serial ports found. Pass the port as a command-line argument.")
    if len(ports) == 1:
        print(f"Auto-selecting: {ports[0].device}")
        return ports[0].device
    for i, p in enumerate(ports):
        print(f"  [{i}]  {p.device}  —  {p.description}")
    choice = input("Select port number (or type a path): ").strip()
    if choice.startswith("/") or choice.startswith("COM"):
        return choice
    return ports[int(choice)].device


# ─────────────────────────────────────────────────────────────────────────────
# SERIAL READER  (daemon thread — never blocks the GUI)
# ─────────────────────────────────────────────────────────────────────────────

class SerialReader(threading.Thread):
    """
    Reads lines from serial in a background thread, parses the 13-field CSV,
    and pushes result dicts onto a thread-safe queue.

    Comment / status lines (starting with '#') are printed to stdout only.
    Bad lines are silently dropped — the GUI never stalls waiting for data.
    """

    def __init__(self, port: str, data_queue: queue.Queue):
        super().__init__(daemon=True, name="SerialReader")
        self.ser = serial.Serial(port, BAUD_RATE, timeout=1)
        self.q   = data_queue

    def run(self):
        while True:
            try:
                raw = self.ser.readline().decode(errors="ignore").strip()
                if not raw:
                    continue
                if raw.startswith("#"):
                    print(raw)
                    continue

                parts = raw.split(",")
                if len(parts) != 13:
                    continue

                d = {
                    "lat":         float(parts[0]),
                    "lon":         float(parts[1]),
                    "target_lat":  float(parts[2]),
                    "target_lon":  float(parts[3]),
                    "heading":     float(parts[4]),
                    "bearing":     float(parts[5]),
                    "motor_pos":   float(parts[6]),
                    "ts_ms":       float(parts[7]),
                    "P":           float(parts[8]),
                    "I":           float(parts[9]),
                    "phase":       int(parts[10]),
                    "altitude":    float(parts[11]),
                    "loiter_step": int(parts[12]),
                }
                self.q.put(d)

            except Exception:
                # Never let a bad line crash the reader thread
                continue


# ─────────────────────────────────────────────────────────────────────────────
# ARROW HELPERS
# ─────────────────────────────────────────────────────────────────────────────

def geo_to_dxdy(heading_deg: float, length: float):
    """
    Geographic heading (0 = North, clockwise) → (dx_lon, dy_lat).
    Used to compute the tip of heading / bearing arrows on the map.
    """
    rad = math.radians(heading_deg)
    return length * math.sin(rad), length * math.cos(rad)


def geo_to_pg_angle(heading_deg: float) -> float:
    """
    Convert geographic heading (0=N, CW) to pyqtgraph ArrowItem angle
    (0=East/right, CCW positive).
    """
    return 90.0 - heading_deg


# ─────────────────────────────────────────────────────────────────────────────
# MAIN WINDOW
# ─────────────────────────────────────────────────────────────────────────────

class Visualizer(QtWidgets.QMainWindow):
    """
    Three-panel real-time display:
      1. GPS map    — trail (phase-colored dots), heading/bearing arrows, target
      2. Altitude   — altitude line, phase-colored dots, threshold ref lines
      3. Control    — P, I, motor position over time
    """

    def __init__(self, data_queue: queue.Queue):
        super().__init__()
        self.q = data_queue
        self.setWindowTitle("BLiMS Car Test Visualizer")

        # ── data ringbuffers (numpy arrays updated in place for speed) ──
        self._cap       = MAX_LEN
        self._n         = 0          # number of valid samples so far
        self._lats      = np.zeros(MAX_LEN)
        self._lons      = np.zeros(MAX_LEN)
        self._ts        = np.zeros(MAX_LEN)   # seconds since boot
        self._headings  = np.zeros(MAX_LEN)
        self._bearings  = np.zeros(MAX_LEN)
        self._motor     = np.zeros(MAX_LEN)
        self._P         = np.zeros(MAX_LEN)
        self._I         = np.zeros(MAX_LEN)
        self._phases    = np.zeros(MAX_LEN, dtype=int)
        self._alts      = np.zeros(MAX_LEN)
        self._loiter    = np.zeros(MAX_LEN, dtype=int)
        self._ptr       = 0          # write pointer (circular)

        self._target_lat = None
        self._target_lon = None

        # ── Qt layout ──────────────────────────────────────────────────
        pg.setConfigOption("background", "#1a1a2e")   # dark navy
        pg.setConfigOption("foreground", "#e0e0e0")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setSpacing(6)
        layout.setContentsMargins(8, 8, 8, 8)

        # ── 1. MAP ─────────────────────────────────────────────────────
        self.map_pw = pg.PlotWidget()
        self.map_pw.setAspectLocked(True)
        self.map_pw.setLabel("left",   "Latitude",  color="#e0e0e0")
        self.map_pw.setLabel("bottom", "Longitude", color="#e0e0e0")
        self.map_pw.showGrid(x=True, y=True, alpha=0.2)
        self.map_pw.getPlotItem().setTitle(
            "Waiting for GPS fix…",
            color="#e0e0e0", size="11pt"
        )
        layout.addWidget(self.map_pw, stretch=3)

        # Full trail line (faint white) — gives spatial context
        self.trail_line = self.map_pw.plot(
            pen=pg.mkPen(color=(255, 255, 255, 40), width=1)
        )

        # Phase-colored scatter dots on top of the trail
        self.trail_dots = pg.ScatterPlotItem(size=5, pxMode=True)
        self.map_pw.addItem(self.trail_dots)

        # Target marker (red triangle)
        self.target_dot = pg.ScatterPlotItem(
            size=16, pxMode=True,
            symbol="t",
            pen=pg.mkPen("r", width=2),
            brush=pg.mkBrush(255, 60, 60, 200)
        )
        self.map_pw.addItem(self.target_dot)

        # Current position (white filled circle)
        self.pos_dot = pg.ScatterPlotItem(
            size=12, pxMode=True,
            symbol="o",
            pen=pg.mkPen("w", width=2),
            brush=pg.mkBrush(255, 255, 255, 230)
        )
        self.map_pw.addItem(self.pos_dot)

        # Heading arrow  (blue line + arrowhead)
        self.head_line = self.map_pw.plot(pen=pg.mkPen("#42a5f5", width=2))
        self.head_arrow = pg.ArrowItem(
            angle=0, tipAngle=28, headLen=14, tailLen=0,
            brush=pg.mkBrush("#42a5f5"),
            pen=pg.mkPen("#42a5f5", width=1)
        )
        self.map_pw.addItem(self.head_arrow)

        # Bearing arrow  (green line + arrowhead)
        self.bear_line = self.map_pw.plot(pen=pg.mkPen("#66bb6a", width=2))
        self.bear_arrow = pg.ArrowItem(
            angle=0, tipAngle=28, headLen=14, tailLen=0,
            brush=pg.mkBrush("#66bb6a"),
            pen=pg.mkPen("#66bb6a", width=1)
        )
        self.map_pw.addItem(self.bear_arrow)

        # Legend (manual, in map corner)
        leg_items = [
            pg.ScatterPlotItem(size=8, symbol="o",
                               brush=PHASE_BRUSH[ph], pen=None,
                               name=PHASE_NAMES[ph])
            for ph in range(7)
        ]
        legend = self.map_pw.addLegend(offset=(-10, 10))
        for ph, item in enumerate(leg_items):
            legend.addItem(item, PHASE_NAMES[ph])
        for item in leg_items:
            self.map_pw.addItem(item)   # keeps legend linked; items at 0,0

        # ── 2. ALTITUDE ───────────────────────────────────────────────
        self.alt_pw = pg.PlotWidget()
        self.alt_pw.setLabel("left",   "Altitude (ft)", color="#e0e0e0")
        self.alt_pw.setLabel("bottom", "Time (s)",       color="#e0e0e0")
        self.alt_pw.showGrid(x=True, y=True, alpha=0.2)
        self.alt_pw.setYRange(-50, 1100)
        layout.addWidget(self.alt_pw, stretch=1)

        self.alt_line = self.alt_pw.plot(
            pen=pg.mkPen("#e0e0e0", width=1.5), name="Alt"
        )
        self.alt_dots = pg.ScatterPlotItem(size=4, pxMode=True)
        self.alt_pw.addItem(self.alt_dots)

        for name, thresh in ALT_THRESHOLDS.items():
            il = pg.InfiniteLine(
                pos=thresh, angle=0,
                pen=pg.mkPen(color=(180, 180, 180, 100), width=1,
                             style=QtCore.Qt.PenStyle.DashLine),
                label=name,
                labelOpts={"color": (180, 180, 180), "position": 0.97,
                           "anchors": [(1, 1), (1, 1)]}
            )
            self.alt_pw.addItem(il)

        # ── 3. CONTROL ────────────────────────────────────────────────
        self.ctrl_pw = pg.PlotWidget()
        self.ctrl_pw.setLabel("left",   "Value",    color="#e0e0e0")
        self.ctrl_pw.setLabel("bottom", "Time (s)", color="#e0e0e0")
        self.ctrl_pw.showGrid(x=True, y=True, alpha=0.2)
        self.ctrl_pw.setYRange(-0.35, 1.05)
        layout.addWidget(self.ctrl_pw, stretch=1)

        ctrl_legend = self.ctrl_pw.addLegend(offset=(-10, 10))
        self.p_line = self.ctrl_pw.plot(
            pen=pg.mkPen("#ffa726", width=1.5), name="P"
        )
        self.i_line = self.ctrl_pw.plot(
            pen=pg.mkPen("#ab47bc", width=1.5), name="I"
        )
        self.motor_line = self.ctrl_pw.plot(
            pen=pg.mkPen(color="#e0e0e0", width=2,
                         style=QtCore.Qt.PenStyle.DashLine),
            name="Motor"
        )
        self.ctrl_pw.addLine(
            y=0.5,
            pen=pg.mkPen(color=(180, 180, 180, 80), width=1,
                         style=QtCore.Qt.PenStyle.DotLine)
        )

        # Link X axes of alt and control plots
        self.ctrl_pw.setXLink(self.alt_pw)

        # ── Timer ──────────────────────────────────────────────────────
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(1000 // GUI_HZ)

        self.resize(1000, 920)
        self.show()

    # ─────────────────────────────────────────────────────────────────────────
    # CIRCULAR BUFFER HELPERS
    # ─────────────────────────────────────────────────────────────────────────

    def _append(self, d: dict):
        """Write one sample into the circular arrays."""
        p = self._ptr
        self._lats[p]     = d["lat"]
        self._lons[p]     = d["lon"]
        self._ts[p]       = d["ts_ms"] / 1000.0
        self._headings[p] = d["heading"]
        self._bearings[p] = d["bearing"]
        self._motor[p]    = d["motor_pos"]
        self._P[p]        = d["P"]
        self._I[p]        = d["I"]
        self._phases[p]   = d["phase"]
        self._alts[p]     = d["altitude"]
        self._loiter[p]   = d["loiter_step"]
        self._ptr         = (p + 1) % self._cap
        if self._n < self._cap:
            self._n += 1
        self._target_lat  = d["target_lat"]
        self._target_lon  = d["target_lon"]

    def _ordered(self):
        """
        Return arrays in chronological order from oldest to newest.
        When the buffer hasn't wrapped yet this is just a slice [0:n].
        After wrapping the write pointer marks the oldest entry.
        """
        n = self._n
        if n < self._cap:
            # not yet full: just return the filled portion
            return (
                self._lats[:n], self._lons[:n], self._ts[:n],
                self._headings[:n], self._bearings[:n],
                self._motor[:n], self._P[:n], self._I[:n],
                self._phases[:n], self._alts[:n]
            )
        # full: roll so that self._ptr (oldest) comes first
        idx = np.roll(np.arange(self._cap), -self._ptr)
        return (
            self._lats[idx], self._lons[idx], self._ts[idx],
            self._headings[idx], self._bearings[idx],
            self._motor[idx], self._P[idx], self._I[idx],
            self._phases[idx], self._alts[idx]
        )

    # ─────────────────────────────────────────────────────────────────────────
    # MAIN UPDATE  (called by QTimer — drains entire queue each tick)
    # ─────────────────────────────────────────────────────────────────────────

    def _update(self):
        # ── drain queue completely ────────────────────────────────────
        got_data = False
        while True:
            try:
                self._append(self.q.get_nowait())
                got_data = True
            except queue.Empty:
                break

        if not got_data or self._n < 2:
            return

        # ── unpack chronological arrays ───────────────────────────────
        (lats, lons, ts,
         headings, bearings, motor, P, I, phases, alts) = self._ordered()

        t0    = ts[0]
        rel_t = ts - t0

        # latest values
        lat      = lats[-1]
        lon      = lons[-1]
        heading  = headings[-1]
        bearing  = bearings[-1]
        phase    = int(phases[-1])
        altitude = alts[-1]
        motor_v  = motor[-1]
        p_val    = P[-1]
        i_val    = I[-1]

        # ── MAP: trail ───────────────────────────────────────────────
        self.trail_line.setData(lons, lats)

        # Phase-colored dots (build spot dicts for ScatterPlotItem)
        spots = [
            {
                "pos": (lons[i], lats[i]),
                "brush": PHASE_BRUSH.get(int(phases[i]), PHASE_BRUSH[0]),
                "pen": None,
                "size": 5,
            }
            for i in range(len(lons))
        ]
        self.trail_dots.setData(spots)

        # Current position
        self.pos_dot.setData([lon], [lat])

        # Target
        if self._target_lat is not None:
            self.target_dot.setData([self._target_lon], [self._target_lat])

        # Heading arrow (blue)
        arrow_len = MAP_PADDING * 0.45
        dx_h, dy_h = geo_to_dxdy(heading, arrow_len)
        tip_hx, tip_hy = lon + dx_h, lat + dy_h
        self.head_line.setData([lon, tip_hx], [lat, tip_hy])
        self.head_arrow.setPos(tip_hx, tip_hy)
        self.head_arrow.setStyle(angle=geo_to_pg_angle(heading))

        # Bearing arrow (green)
        dx_b, dy_b = geo_to_dxdy(bearing, arrow_len)
        tip_bx, tip_by = lon + dx_b, lat + dy_b
        self.bear_line.setData([lon, tip_bx], [lat, tip_by])
        self.bear_arrow.setPos(tip_bx, tip_by)
        self.bear_arrow.setStyle(angle=geo_to_pg_angle(bearing))

        # Map bounds centered on target
        if self._target_lat is not None:
            self.map_pw.setXRange(
                self._target_lon - MAP_PADDING,
                self._target_lon + MAP_PADDING,
                padding=0
            )
            self.map_pw.setYRange(
                self._target_lat - MAP_PADDING,
                self._target_lat + MAP_PADDING,
                padding=0
            )

        # Title
        phase_name = PHASE_NAMES[phase] if 0 <= phase < len(PHASE_NAMES) else "???"
        loiter_info = ""
        if phase == 6:
            ls = int(self._loiter[(self._ptr - 1) % self._cap])
            loiter_info = f"  [{LOITER_NAMES[ls]}]" if 0 <= ls < len(LOITER_NAMES) else ""
        r, g, b = PHASE_RGB.get(phase, (200, 200, 200))
        hex_color = f"#{r:02x}{g:02x}{b:02x}"
        self.map_pw.getPlotItem().setTitle(
            f"<span style='color:{hex_color};font-weight:bold'>{phase_name}{loiter_info}</span>"
            f"  |  Alt: <b>{altitude:.0f} ft</b>"
            f"  |  Motor: <b>{motor_v:.3f}</b>"
            f"  |  Head: <b>{heading:.0f}°</b>"
            f"  Bear: <b>{bearing:.0f}°</b>"
            f"  |  P={p_val:.3f}  I={i_val:.3f}",
            size="10pt"
        )

        # ── ALTITUDE ──────────────────────────────────────────────────
        self.alt_line.setData(rel_t, alts)

        alt_spots = [
            {
                "pos": (rel_t[i], alts[i]),
                "brush": PHASE_BRUSH.get(int(phases[i]), PHASE_BRUSH[0]),
                "pen": None,
                "size": 4,
            }
            for i in range(len(rel_t))
        ]
        self.alt_dots.setData(alt_spots)

        # ── CONTROL ───────────────────────────────────────────────────
        self.p_line.setData(rel_t, P)
        self.i_line.setData(rel_t, I)
        self.motor_line.setData(rel_t, motor)

        # Slide the time window (alt and ctrl share X via setXLink)
        x_min = rel_t[-1] - WINDOW_SECONDS if rel_t[-1] > WINDOW_SECONDS else 0
        x_max = rel_t[-1] + 2
        self.alt_pw.setXRange(x_min, x_max, padding=0)


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    port = find_port()
    print(f"Opening {port} at {BAUD_RATE} baud …")

    data_queue = queue.Queue()

    try:
        reader = SerialReader(port, data_queue)
    except serial.SerialException as e:
        sys.exit(f"Cannot open {port}: {e}")

    reader.start()
    print("Serial reader started (daemon thread).")
    print("Waiting for GPS data…\n")

    app = QtWidgets.QApplication(sys.argv)
    win = Visualizer(data_queue)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()