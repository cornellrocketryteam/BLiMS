import serial
import pty
import os
import threading

# Create two virtual ports
master_a, slave_a = pty.openpty()
master_b, slave_b = pty.openpty()

print(f"Virtual port A: {os.ttyname(slave_a)}")
print(f"Virtual port B: {os.ttyname(slave_b)}")

ser = serial.Serial('/dev/cu.usbmodem1101', 115200)

def read_and_forward():
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            os.write(master_a, data)
            os.write(master_b, data)

t = threading.Thread(target=read_and_forward, daemon=True)
t.start()
print("Splitting serial — press Ctrl+C to stop")
t.join()