# uavRPYvisualizer

A MATLAB app for visualizing **roll**, **pitch**, and **yaw** dynamics on a standard X-frame quadcopter. Drive the four motors individually with sliders, or click a demo button to watch the UAV execute a clean, isolated rotation. Built on quaternion attitude dynamics with a stabilizing PD controller.


## What it does

You set four motor speeds (rad/s); the app converts the resulting thrust differentials into a commanded attitude, runs a PD controller against the current orientation, integrates the rigid-body rotational dynamics, and renders the quadcopter at a fixed altitude with rotors spinning in their correct directions. It's meant for teaching, intuition-building, and quick sanity checks of the X-frame mixing convention.

Visually, the UAV is color-coded so the geometry is unambiguous:

- **Front motors (1, 2)** — blue rotor disks
- **Rear motors (3, 4)** — green rotor disks
- **Red nose marker** — indicates the front (+X body) direction


## Motor layout (X-frame)

```
        FRONT (red nose)
              ^
              |
      M1 ●         ● M2
       (FL,CW)   (FR,CCW)

       (RL,CCW)  (RR,CW)
      M4 ●         ● M3
```

| Motor | Position    | Spin direction | Color |
|-------|-------------|----------------|-------|
| 1     | Front-Left  | CW             | Blue  |
| 2     | Front-Right | CCW            | Blue  |
| 3     | Rear-Right  | CW             | Green |
| 4     | Rear-Left   | CCW            | Green |

Diagonally opposite motors spin the same direction so reaction torques cancel in hover — the standard X-frame convention.


## Requirements

- **MATLAB R2025a or later** (uses `uigridlayout`, `uislider`, `hgtransform`, `makehgtform`, `timer`).
- No additional toolboxes required.
- Compatible with **Windows**, **macOS**, and **Linux**.

## Installation

### Option 1: MATLAB File Exchange

Install from the [File Exchange page](https://www.mathworks.com/matlabcentral/fileexchange/183084-uavrpyvisualizer) using the **Toolbox** download, or click **Open in MATLAB Online** to try it without installing.

### Option 2: Manual

1. Download or clone this repository.
2. From MATLAB, `cd` into the folder containing `uavRPYvisualizer.m`, or add it to the path:
   ```matlab
   addpath('path/to/uavRPYvisualizer');
   ```

## Usage

Launch the app:

```matlab
app = uavRPYvisualizer;
```

### Controls (left panel)

| Control                       | What it does                                                              |
|-------------------------------|---------------------------------------------------------------------------|
| **Start**                     | Begins the simulation timer (~50 Hz update).                              |
| **Pause**                     | Stops the timer; state is preserved.                                      |
| **Reset**                     | Resets attitude to identity, clears demo mode.                            |
| **Demo: Roll / Pitch / Yaw**  | Plays a canned motor pattern that produces an isolated rotation.          |
| **Stop Demo (Manual Control)**| Returns to manual slider control.                                         |
| **Time step dt (s)**          | Integration step. Default `0.01`. Range `[0.001, 0.05]`.                  |
| **Motor sliders ω₁..ω₄**      | Motor speeds in rad/s. Range `[0, 1200]`. Default `700` (hover-ish).      |

### Readouts

Live roll (φ), pitch (θ), and yaw (ψ) angles are displayed in the left panel in degrees, computed from the current quaternion via a ZYX Euler decomposition.

### Trying it manually

1. Click **Start**.
2. Increase motors `2` and `3`, decrease `1` and `4` — the UAV rolls.
3. Increase the rear pair (`3`, `4`), decrease the front pair (`1`, `2`) — the nose pitches up.
4. Increase the CW pair (`1`, `3`), decrease the CCW pair (`2`, `4`) — the UAV yaws.


## How it works

### Mixing: motor speeds → commanded attitude

Motor speeds are squared (thrust ∝ ω²) and combined into roll/pitch/yaw signals:

```
pitchSig = (ω₁² + ω₂²) − (ω₃² + ω₄²)     % front − rear
rollSig  = (ω₁² + ω₄²) − (ω₂² + ω₃²)     % left  − right
yawSig   = (ω₁² + ω₃²) − (ω₂² + ω₄²)     % CW    − CCW
```

These are scaled by command gains and clamped to bounded "normal-flight" envelopes:

- Roll: ±20°
- Pitch: ±20°
- Yaw: ±35°

This bounding is what keeps the visualization tame and pedagogically useful — you can't accidentally flip the UAV.

### Control: PD on attitude error

A simple PD law produces a body-frame torque:

```
τ = Kp · (attitudeCmd − attitudeActual) − Kd · ω_body
```

with `Kp = diag([2.0, 2.0, 1.2])`, `Kd = diag([0.25, 0.25, 0.20])`, and per-axis torque saturation `[1.0, 1.0, 0.6] N·m`.

### Dynamics: quaternion + RK2

The rotational equations of motion

```
ω̇ = I⁻¹ (τ − ω × Iω)
q̇ = ½ Ω(ω) q
```

are integrated with a midpoint (RK2) scheme, with `I = diag([0.02, 0.02, 0.04]) kg·m²`. The quaternion is renormalized every step; if it ever goes non-finite the state resets safely.

### Rendering

- The quadcopter mesh is built from two rotated arm boxes plus a hub; rotors are translucent disks.
- The quaternion is converted to a rotation matrix and applied to a top-level `hgtransform`. A `diag([1 1 -1])` similarity transform handles the body-vs-display Z convention so altitude stays positive in the rendered scene.
- Rotors spin visually at a rate proportional to ω, with direction set by the `yawSign = [+1, −1, +1, −1]` pattern.

---

## File structure

```
uavRPYvisualizer.m   % single-file App (matlab.apps.AppBase subclass)
README.md
LICENSE
```

The whole app lives in one class — UI construction (`createUI`), graphics (`buildQuadGraphics`), control + dynamics (`motorSpeedsToStabilizedTorque`, `stepAttitudeRK2`), and rendering (`updateGraphics`) are all methods on `uavRPYvisualizer`.


## Tuning

If you want to experiment with the dynamics, edit `quadParams` near the top of the class:

| Parameter        | Meaning                                   | Default                  |
|------------------|-------------------------------------------|--------------------------|
| `L`              | Arm length (m)                            | `0.25`                   |
| `I`              | Body inertia tensor (kg·m²)               | `diag([0.02 0.02 0.04])` |
| `wMax`           | Max motor speed (rad/s)                   | `1200`                   |
| `maxRollDeg`     | Roll envelope (deg)                       | `20`                     |
| `maxPitchDeg`    | Pitch envelope (deg)                      | `20`                     |
| `maxYawDeg`      | Yaw envelope (deg)                        | `35`                     |
| `Kp`, `Kd`       | PD gains                                  | see source               |
| `tauMax`         | Per-axis torque saturation (N·m)          | `[1.0; 1.0; 0.6]`        |

Lift the angle envelopes if you want acrobatic behavior — but expect the small-angle assumptions in the mixer to start showing.


## Citation

If you use this in coursework, papers, or talks, please cite:

> Phadke, A. (2026). *uavRPYvisualizer*. MATLAB Central File Exchange. https://www.mathworks.com/matlabcentral/fileexchange/183084-uavrpyvisualizer


## Author

**Abhishek Phadke** — [MATLAB Central profile](https://www.mathworks.com/matlabcentral/profile/authors/18521943)


## License

See the license file included with the File Exchange submission.

## Tags

`aerospace` · `physics` · `uav` · `quadcopter` · `attitude-control` · `quaternion` · `matlab-app`
