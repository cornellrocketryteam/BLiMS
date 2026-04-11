"""
BLIMS Flight Visualizer
-----------------------
Based off of car_test_visualizer which utilizes PyQtGraph to visualize the 
GPS data and control outputs from the BLiMS car test. This visualizer will be used 
for flight testing (L3), and will be updated to include more relevant information 
such as altitude, airspeed, attitude, etc. depending on what data we are able to log 
from the flight controller during testing.
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


