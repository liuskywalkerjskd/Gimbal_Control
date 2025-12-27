# Gimbal Control - 3D Visualization System

[中文版](./README_zh.md) | English

## Overview

Gimbal Control is an interactive 3D visualization system for simulating gimbal stabilization systems with real-time inverse kinematics (IK) computation. This tool provides a comprehensive platform for understanding and testing gimbal behavior under various conditions, including dynamic base disturbances and pitch angle constraints.

## Features

- **Real-time 3D Visualization**: Interactive matplotlib-based 3D rendering with customizable views
- **Inverse Kinematics Solver**: Automatic computation of gimbal joint angles to achieve target orientations
- **Dynamic Base Simulation**: Simulates platform motion with sine wave disturbances
- **Pitch Limit Constraints**: Configurable pitch angle limits with visual feedback
- **Interactive Controls**: 
  - Real-time sliders for adjusting target and base attitudes (Yaw, Pitch, Roll)
  - Reset button to restore initial state
  - Sine wave mode for automatic base motion simulation
- **Visual Feedback**:
  - Color-coded status indicators (optimal/limited states)
  - Motion trails showing sensor movement history
  - Ghost overlay showing target orientation
  - Beam projection from sensor
  - Multiple coordinate frame displays (World, Base, Yaw Joint, Sensor)
- **Performance Metrics**: Real-time display of tracking error and joint states

## Technical Details

### Gimbal Configuration
- Two-axis gimbal system (Yaw + Pitch)
- Configurable link lengths (`L_BY`, `L_YP`)
- Default pitch limit: ±45 degrees
- Sensor visualization as 3D cuboid

### Coordinate Frames
- **World Frame**: Global reference frame
- **Base Frame**: Platform/vehicle reference frame
- **Yaw Joint Frame**: First rotation axis
- **Sensor Frame**: End-effector frame (actuated orientation)

### Mathematical Model
The system implements a geometric inverse kinematics solution:
1. Converts target orientation from world frame to base frame
2. Computes yaw and pitch angles using arctangent functions
3. Applies pitch constraints to ensure mechanical limits are respected
4. Uses ZYX Euler angle convention for all rotations

## Installation

### Prerequisites
- Python 3.7+
- pip (Python package manager)

### Required Dependencies
```bash
pip install matplotlib numpy scipy
```

### Quick Start
```bash
# Clone the repository
git clone https://github.com/liuskywalkerjskd/Gimbal_Control.git
cd Gimbal_Control

# Install dependencies
pip install -r requirements.txt  # or manually install: pip install matplotlib numpy scipy

# Run the visualization
python axis-visual.py
```

## Usage

### Basic Operation
1. **Launch the application**: Run `python axis-visual.py`
2. **Adjust target attitude**: Use the top sliders (cyan) to set desired sensor orientation
3. **Adjust base attitude**: Use the middle sliders (orange) to set platform orientation
4. **View system state**: Check the right panel for joint angles, errors, and status

### Interactive Features

#### Sliders
- **Target Attitude** (Cyan):
  - `Yaw`: -180° to +180°
  - `Pitch`: -90° to +90°
  - `Roll`: -180° to +180°
- **Base Attitude** (Orange):
  - `Yaw`: -180° to +180°
  - `Pitch`: -90° to +90°
  - `Roll`: -180° to +180°

#### Buttons
- **RESET**: Restores all angles to zero
- **SINE WAVE**: Toggles automatic base motion simulation
  - Creates realistic platform disturbance
  - Amplitude: 15° (configurable)
  - Frequency: 1 Hz (configurable)

#### Visual Indicators
- **Green status**: System operating within limits
- **Red status**: Pitch limit reached
- **Cyan links**: Normal operation
- **Red links**: Pitch constraint active
- **Yellow trail**: Recent motion path
- **Ghost sensor**: Target orientation overlay

### Configuration

Edit the `CONFIG` dictionary in `axis-visual.py` to customize:
```python
CONFIG = {
    'L_BY': 1.5,              # Base to yaw joint length
    'L_YP': 1.5,              # Yaw joint to pitch/sensor length
    'PITCH_LIMIT': 45.0,      # Maximum pitch angle (degrees)
    'SENSOR_SIZE': [0.6, 0.4, 0.3],  # Sensor dimensions [length, width, height]
    'WAVE_AMP': 15.0,         # Sine wave amplitude (degrees)
    'WAVE_FREQ': 1.0,         # Sine wave frequency (Hz)
    'COLORS': { ... }         # Color scheme
}
```

## Use Cases

- **Education**: Learn gimbal mechanics and inverse kinematics
- **System Design**: Evaluate gimbal configurations and constraints
- **Control Testing**: Validate stabilization algorithms
- **Performance Analysis**: Assess tracking accuracy under disturbances
- **Visualization**: Create demonstrations of gimbal behavior

## Technical Notes

### Rotation Convention
- Uses ZYX Euler angles (Yaw-Pitch-Roll)
- All angles in degrees for user interface
- Internal calculations use radians
- Rotation matrices computed using scipy.spatial.transform.Rotation

### Coordinate System
- Right-handed coordinate system
- Z-axis: Up
- X-axis: Forward
- Y-axis: Left

### Performance
- Animation refresh rate: 50ms (20 FPS)
- Trail history: Last 50 positions
- Optimized for real-time interaction

## File Structure

```
Gimbal_Control/
├── axis-visual.py          # Main visualization application
├── Gimbal_Control _V1.1.pdf  # Documentation (Chinese)
├── README.md               # This file (English)
└── README_zh.md            # Chinese version
```

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

This project is available for educational and research purposes.

## Author

Created by liuskywalkerjskd

## References

- [Gimbal Documentation (Chinese)](./Gimbal_Control%20_V1.1.pdf)

---

For the Chinese version of this README, see [README_zh.md](./README_zh.md)
