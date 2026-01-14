# IMU to Robot Axis Mapping Guide

## Current Setup (As of Jan 2026)
Based on your glove layout, the BNO085 sensor is mounted 90° rotated relative to the "standard" forward direction.

### Physical Mapping
| User Action (Glove) | BNO Sensor Readings | Mapped to Robot Axis |
|---------------------|---------------------|----------------------|
| **Tilt Forward/Back** | **Roll** | **X Axis** (Forward/Back) |
| **Tilt Left/Right** | **Pitch** | **Y Axis** (Left/Right) |
| **Twist Wrist** | **Yaw** | **Rotation (Ry/Shake Head)** |
| **Nod Hand (Fwd/Back)** | **Roll** | **Rotation (Rz/Tilt)** |
| **Tilt Hand (Side-to-Side)** | **Pitch** | **Roll (Rx/Tilt)** |

---

## 3D Cube (Visualizer) Mapping
The 3D cube axes are now synchronized with your glove:

| Movement | Cube Axis | Sensor Axis | Current Invert |
|----------|-----------|-------------|----------------|
| **Nod (Down/Up)** | **X** | **Pitch** | `False` |
| **Shake Head (L/R)** | **Y** | **Yaw** | `True` |
| **Tilt (Ear to Shoulder)** | **Z** | **Roll** | `False` |

> [!TIP]
> This configuration ensures that tilting your hand forward makes the cube nod forward, and twisting your wrist shakes the head correctly. 

## How to Configure
All settings are located in:  
`config/constants.py`

Look for the `IMU_AXIS_MAPPING` dictionary:

```python
IMU_AXIS_MAPPING = {
    # Forward/Back Movement
    'x_axis': 'roll',      # Uses Glove Forward/Back
    'x_invert': False,     # Change to True if Forward moves Backward

    # Left/Right Movement
    'y_axis': 'pitch',     # Uses Glove Left/Right
    'y_invert': False,     # Inverted back (Left moves Left)
    
    # ...
}
```

## Troubleshooting Directions

1.  **Run the visualizer:** `./run.sh`
2.  **Tilt Glove Forward:** 
    *   If Robot moves **Backward** -> Set `'x_invert': False` (Toggle it)
3.  **Tilt Glove Left:**
    *   If Robot moves **Right** -> Set `'y_invert': True` (Toggle it)
4.  **Tilt Glove Up:**
    *   If Robot moves **Down** -> Set `'z_invert': True` (in `z_invert` section)

## Why did we change this?
Original code assumed `Roll = X` and `Pitch = Y`. However, due to the sensor mounting on your specific glove, the axes were swapped. We updated the mapping to align the **Physical Gesture** with the **Robot Movement**.
