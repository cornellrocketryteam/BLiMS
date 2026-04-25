# odrive_init.py — run before every test session
import odrive
from odrive.enums import *

print("Finding ODrive...")
odrv0 = odrive.find_any()
print(f"Found: {odrv0.serial_number}")

# Verify PWM mapping is still intact
mode = odrv0.config.gpio8_mode
endpoint = odrv0.config.gpio8_pwm_mapping.endpoint
print(f"gpio8_mode: {mode} (expect 10)")
print(f"gpio8_pwm endpoint: {endpoint} (expect axis0.controller.input_pos)")

if mode != 10 or endpoint != "axis0.controller.input_pos":
    print("Config lost — reapplying...")
    odrv0.config.gpio8_mode = GpioMode.PWM  # = 10
    odrv0.config.gpio8_pwm_mapping.endpoint = odrv0.axis0.controller._input_pos_property
    odrv0.config.gpio8_pwm_mapping.min = -21
    odrv0.config.gpio8_pwm_mapping.max = 21
    odrv0.save_configuration()  # ODrive reboots here
    print("Saved. Reconnecting...")
    odrv0 = odrive.find_any()

# Arm motor
odrv0.axis0.requested_state = AxisState.CLOSED_LOOP_CONTROL
print("ODrive armed. Ready.")