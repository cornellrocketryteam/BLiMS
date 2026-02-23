# BLiMS Simulation Test Framework

Quick testing of BLiMS landing logic without a physical car test.

## Overview

The simulation framework allows you to:
- Test phase transitions (TRACK → DOWNWIND → BASE → FINAL → NEUTRAL → LOITER)
- Verify PI controller behavior with altitude changes
- Simulate loiter state machine
- Generate CSV output for visualization and analysis
- Rapid debug cycles (< 1 second compile + run)

## Files

- **sim_test.hpp / sim_test.cpp**: Core `SimData` class with altitude data + state tracking
- **sim_runner.cpp**: Standalone simulator that reads altitude, runs landing logic
- **CMakeLists.txt**: Build configuration

## Quick Start

### Simple (no CMake)
```bash
cd /Users/willchen/Rocketry/BLiMS/sim
g++ -std=c++17 sim_runner.cpp sim_test.cpp -o sim_runner && ./sim_runner
```

### With CMake
```bash
cd /Users/willchen/Rocketry/BLiMS/sim
mkdir -p build
cd build
cmake ..
make
./sim_runner
```

## Output

The simulation produces:
1. **Console output** - Phase transitions and key checkpoints
2. **sim_output.csv** - Full state log (open in Excel/Python for plots)

### CSV Columns
```
sample              - Index in altitude array
timestamp_ms        - Simulated time (ms)
altitude_ft         - Altitude from flight data
phase               - Landing phase (0=HELD, 1=TRACK, 2=DOWNWIND, 3=BASE, 4=FINAL, 5=NEUTRAL, 6=LOITER)
motor_pos           - Normalized motor position (0.3-0.7)
heading_error_deg   - Simulated heading error
p_term              - Proportional controller output
i_term              - Integral controller accumulator
loiter_step         - Loiter sub-state (0=TURN_RIGHT, 1=PAUSE_RIGHT, 2=TURN_LEFT, 3=PAUSE_LEFT)
```

## Using SimData in Your Own Tests

```cpp
#include "sim_test.hpp"

SimData sim;

// Access altitude
float alt = sim.get_alt();

// Check if more data
if (sim.has_more_data()) { ... }

// Access current state
printf("Phase: %s\n", SimData::phase_name(sim.current_state.phase));
printf("Motor: %.3f\n", sim.current_state.motor_position);

// Log human-readable state
sim.log_state("MY_TEST");

// Reset for another run
sim.reset();
```

## Customizing Simulation

### Change Altitude Thresholds
Edit in `sim_runner.cpp`:
```cpp
const float DOWNWIND_ALT = 1000.0f;  // 1000 ft AGL
const float BASE_ALT = 600.0f;       // 600 ft AGL
const float FINAL_ALT = 300.0f;      // 300 ft AGL
const float NEUTRAL_ALT = 100.0f;    // 100 ft AGL
```

### Change PI Controller Gains
Edit in `sim_runner.cpp`:
```cpp
const float Kp = 0.009f;             // Proportional gain
const float Ki = 0.001f;             // Integral gain
const float integral_max = 10.0f;    // Integral windup limit
```

### Change Heading Error Simulation
Edit the `simulate_heading_error()` function:
```cpp
float simulate_heading_error(int timestamp_ms) {
    // Change this to your desired error pattern
    return 30.0f * sin(timestamp_ms / 1000.0f);
}
```

## Analyzing Results

### In Python
```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('sim_output.csv')

# Plot altitude vs phase
plt.scatter(df['altitude_ft'], df['phase'], s=1)
plt.xlabel('Altitude (ft)')
plt.ylabel('Phase')
plt.title('Landing Phase vs Altitude')
plt.show()

# Plot motor control
plt.plot(df['timestamp_ms']/1000, df['motor_pos'])
plt.xlabel('Time (s)')
plt.ylabel('Motor Position')
plt.title('Motor Control Over Time')
plt.show()
```

### In Excel
1. Open `sim_output.csv`
2. Insert charts:
   - Line chart: Time vs Motor Position
   - Scatter plot: Altitude vs Phase
   - Line chart: Time vs Heading Error

## Integration with Car Tests

After verifying logic in simulation:
1. Run `sim_runner` to check phase transitions
2. Verify motor control looks reasonable
3. Check CSV for any anomalies
4. Then run actual car test with same parameters

This saves time and battery! 🔋

## Notes

- Altitude data comes from real flight (5,538 samples)
- Simulated heading error uses sine wave (edit for your pattern)
- Motor position limited to [0.3, 0.7] normalized range
- All timing in milliseconds (10ms per sample)
