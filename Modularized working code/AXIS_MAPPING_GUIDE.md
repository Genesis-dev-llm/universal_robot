# IMU Control Mapping Guide

## Physical Gesture → Control Action

### Your Setup
- **Visualization**: Cube is "in front of you", facing away (mirror-style)
- **BNO085 Mounting**: 90° rotated on glove

---

## Mode Mappings

| Mode | Gesture | Effect |
|------|---------|--------|
| **1: CRANE** | Nod forward/back | TCP moves **away**/toward you |
| **1: CRANE** | Tilt left/right | Base rotates left/right (crane swing) |
| **2: VERTICAL** | Nod forward | TCP moves **up** |
| **2: VERTICAL** | Nod back | TCP moves **down** |
| **3: LATERAL** | Tilt left | TCP moves **left** (your perspective) |
| **3: LATERAL** | Tilt right | TCP moves **right** (your perspective) |
| **4: WRIST 1&2** | Tilt left/right | Wrist 1 rotates |
| **4: WRIST 1&2** | Nod forward/back | Wrist 2 rotates |
| **5: WRIST 3** | Tilt left/right | Wrist 3 (screwdriver) rotates |
| **6: ORIENT MIMIC** | Full orientation | TCP mimics your hand orientation |

---

## Technical Mapping (coordinate_frames.py)

### Translation (Modes 1-3)
```
Robot X ← -roll   (nod: forward→away, back→toward)
Robot Y ← +pitch  (tilt: left→left, right→right)  
Robot Z ← +pitch  (Mode 2: forward→up)
```

### Rotation (Modes 4-6)
```
Robot RX ← -pitch (tilt left/right - INVERTED)
Robot RY ← -yaw   (wrist twist)
Robot RZ ← +roll  (nod forward/back)
```

### Visualization (Cube)
```
Cube X ← -pitch  (nod forward → cube nods away)
Cube Y ← +yaw    (look left → cube looks left from your view)
Cube Z ← +roll   (tilt → cube tilts same direction)
```

---

## Inversion Summary

| Axis | Old Sign | New Sign | Reason |
|------|----------|----------|--------|
| Translation X (Mode 1) | +1 | **-1** | Forward tilt → move away |
| Translation Y (Mode 1, 3) | -1 | **+1** | Tilt left → move left |
| Rotation RX (Modes 4,5,6) | +1 | **-1** | Tilt left → rotate correct way |
| Viz X | +1 | **-1** | Mirror-style cube facing |
| Viz Y | -1 | **+1** | Mirror-style cube facing |
