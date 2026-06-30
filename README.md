# Attitude Controller Simulator

A C++98 framework for simulating closed-loop 3D attitude control of rigid bodies. The project provides a deterministic execution environment for control algorithm development and validation under realistic physical and sensor constraints. The simulator follows MISRA C++:2008 coding standards and features a modular architecture that separates physics, control, sensor, and actuator models for independent testing and integration with a Python package for post-processing and visualization.

Runs are bit-reproducible for a given RNG seed. The only non-deterministic element is Gaussian sensor noise; dynamics, PID state, the delay queue, and setpoint scheduling produce the same numbers every time.

---

## System Model

### Rotational Dynamics

Rigid body, rotation about a fixed center of mass, diagonal inertia $I = \mathrm{diag}(I_{xx},I_{yy},I_{zz})$ (principal axes only). Euler's equation with the gyroscopic term:

$$
I\,\dot{\boldsymbol\omega} = \boldsymbol\tau - \boldsymbol\omega \times (I\,\boldsymbol\omega)
$$

$\boldsymbol\tau$ is the torque the actuator is currently holding. $I^{-1}$ is cached at initialization; angular acceleration is a component-wise multiply: $\dot{\boldsymbol\omega} = I^{-1}(\boldsymbol\tau - \boldsymbol\omega \times I\boldsymbol\omega)$.

### Attitude Kinematics

3-2-1 (ZYX) Euler sequence — yaw $\psi$, pitch $\theta$, roll $\phi$. Kinematic map from body rates $(p,q,r)$ to angle rates:

$$
\begin{bmatrix} \dot\phi \\ \dot\theta \\ \dot\psi \end{bmatrix}
=
\begin{bmatrix}
p + (q\sin\phi + r\cos\phi)\tan\theta \\
q\cos\phi - r\sin\phi \\
(q\sin\phi + r\cos\phi)/\cos\theta
\end{bmatrix}
$$

The singularity at $\theta = \pm\pi/2$ is inherent to any Euler-angle representation. The 3-2-1 sequence was chosen for its direct correspondence to the attitude control literature and to the standard pilot-convention (roll, pitch, yaw); a quaternion representation would remove the singularity but would decouple the state variables from their physical meaning in a way that complicates both the control law and the output analysis. For a testbed operating within a normal flight envelope, the arithmetic guard suffices: $|\cos\theta|$ floored to $10^{-6}$ with sign preserved, preventing a divide-by-zero without any added kinematic machinery. Angles wrapped to $[-\pi, \pi]$ after each step.

### Numerical Integration

Semi-implicit Euler at fixed $\Delta t$: angular velocity updated first from the old angles, then kinematics run with the fresh rate:

$$
\boldsymbol\omega_{k+1} = \boldsymbol\omega_k + \Delta t\,I^{-1}\!\left(\boldsymbol\tau_k - \boldsymbol\omega_k \times I\boldsymbol\omega_k\right),
\qquad
\boldsymbol\Theta_{k+1} = \boldsymbol\Theta_k + \Delta t\,f\!\left(\boldsymbol\Theta_k,\,\boldsymbol\omega_{k+1}\right)
$$

Default $\Delta t = 0.01\ \text{s}$ (100 Hz). Any value in $(0,1]\ \text{s}$ is accepted; warnings above $0.1\ \text{s}$ or when $\Delta t > T/10$.

### Control Law

Three independent PID controllers, one per axis, on error $e_k = r_k - \tilde y_k$. Two details beyond the basic form.

The derivative uses a first-order EMA rather than the raw backward difference:

$$
d_k = \alpha\,d_{k-1} + (1-\alpha)\,\frac{e_k - e_{k-1}}{\Delta t}, \qquad \alpha \in [0,1]
$$

$\alpha = 0$ gives the raw difference; normal-case config uses 0.4.

The integrator is a leaky integrator, not a plain accumulator — backward-Euler solution of $\dot I = e - I/\tau_\text{eff}$, with $\tau_\text{eff} = \max(\tau,\,10\Delta t)$. Unsaturated update:

$$
I_{k+1} = \frac{I_k + \Delta t\,e_k}{1 + \Delta t/\tau_{\mathrm{eff}}}
$$

When saturation binds (tentative output $u^* = K_p e_k + K_i I_{k+1} + K_d d_k$ hitting $\pm 1000$), back-calculation corrects the integral against the clipped output $u_\text{sat}$:

$$
I_{k+1} = \frac{I_k + \Delta t\,e_k + \dfrac{\Delta t}{\tau_{\mathrm{eff}}}\cdot\dfrac{u_{\mathrm{sat}} - K_p e_k - K_d d_k}{K_i}}{1 + \Delta t/\tau_{\mathrm{eff}}}
$$

Integral clamped to $[-100, 100]$; output to $[-1000, 1000]$.

When `RateFeedback` is enabled in the XML config, the derivative path is replaced by the measured body rate. For a constant setpoint, $\dot e \approx -\omega$, so the D-term becomes $-K_d\,\tilde\omega$ instead of the EMA-filtered finite difference:

$$
u_k = K_p e_k + K_i I_k - K_d\,\tilde\omega_k
$$

Same $K_p$, $K_i$, $K_d$ and the same leaky integrator with back-calculation; only the derivative source changes. `SimulationManager` reads `getMeasuredOmega()` when the flag is set.

### Sensor Model

True attitude plus integrated drift plus white noise:

$$
\tilde y_k = y_k + b_k + n_k, \qquad n_k \sim \mathcal N(0,\sigma^2), \qquad b_{k+1} = b_k + \dot b\,\Delta t
$$

Noise drawn per axis and per call via Box-Muller, with a cached spare deviate to halve the trig cost. Each channel (`getMeasuredOrientation`, `getMeasuredOmega`) draws fresh noise independently. Drift is deterministic.

### Actuator Model

Two conditioning stages before the delay queue: per-axis symmetric clamp, then a magnitude cap that rescales without rotating:

$$
\boldsymbol\tau' = \mathrm{clamp}(\boldsymbol\tau,\,\boldsymbol\tau^{\max}_\text{axis}), \qquad
\boldsymbol\tau'' = \boldsymbol\tau' \cdot \min\!\left(1,\,\frac{\tau^{\max}_\text{mag}}{\|\boldsymbol\tau'\|}\right)
$$

The conditioned command is timestamped, pushed to a FIFO. `update()` advances the actuator clock by $\Delta t$ and drains every entry with $t_\text{issue} + \delta \le t_\text{now}$. The last released torque is held constant until the next — zero-order hold — and that's what the integrator sees.

### Setpoint Scheduling

Piecewise-constant reference: a list of (time, roll, pitch, yaw) waypoints, zero-order held. Active setpoint at time $t$ is the last entry with timestamp $\le t$. Each switch is a simultaneous step command on all three axes.

---

## Simulation Loop and Safety Monitor

`SimulationManager` drives the loop. `stepOnce()` runs in strict order:

1. **Validation** — finite-value check on every module
2. **Sensing** — drift integrated; noisy orientation sampled
3. **Control** — setpoint resolved; PID torque from attitude only, or with measured $\tilde\omega$ when `RateFeedback` is set
4. **Dispatch** — command conditioned, queued
5. **Actuation** — actuator clock advanced; due commands released
6. **Integration** — rigid body stepped with the held torque
7. **Logging** — state row appended to the in-memory buffer
8. **Advance** — clock ticks; run ends at the configured deadline

`validateState` runs before every step. Three error conditions: `ERR_RT_NAN_DETECTED` (non-finite anywhere) is a hard halt. `ERR_RT_ATTITUDE_LIMIT_EXCEEDED` and `ERR_RT_CONTROL_DIVERGENCE` clear the PID integral states via `controller_.reset()` and let execution continue. `ERR_RT_CONTROL_SATURATION` (queued torque magnitude above threshold, default 100) ends the run with a warning and exit code 0.

At the end, the trajectory lands in:

```
time, pitch, yaw, roll, omega_x, omega_y, omega_z,
torque_x, torque_y, torque_z, gyro_x, gyro_y, gyro_z
```

One row per step. Angles in rad, rates in rad/s, the control torque $\boldsymbol\tau$ (last torque applied by the actuator) and the gyroscopic reaction $\boldsymbol\omega \times (I\boldsymbol\omega)$ in N·m, both body-frame. This file is the only interface to the Python layer.

### Coordinate Convention

One axis ordering throughout: $(x, y, z) = (\text{roll, pitch, yaw})$, body rates $(p, q, r)$ matching. Gain vectors, inertia, initial conditions, noise, torque limits, PID bank, actuator commands, dynamics state, exported CSV — all the same order, no permutation anywhere. The Python side reads columns by name and builds the body-to-inertial matrix as $R_z(\psi)\,R_y(\theta)\,R_x(\phi)$.

---

## Software Architecture

Nine modules, single responsibilities, injected dependencies.

| Module | What it does |
|---|---|
| `Vector3f` | Three-component float algebra: dot, cross, component-wise multiply, norm, finiteness |
| `RigidBodySimulator` | Euler dynamics and 3-2-1 kinematic integration |
| `PID` | Single-axis discrete PID: EMA derivative, leaky integrator with back-calculation |
| `PIDController` | Three `PID` instances, shared smoothing and anti-windup constant |
| `GaussianNoiseGenerator` | Box-Muller with cached spare |
| `SensorSimulator` | Drift integration, orientation and rate measurement |
| `ActuatorDriver` | Saturation conditioning, FIFO-delayed zero-order hold |
| `InputParser` | XML and plain-text config loading, validation |
| `SimulationManager` | Fixed-step loop, safety monitor, log, CSV export |

Per-step data flow:

```
InputParser ──▶ SimulationManager
                    │  setpoint
                    ▼
    SensorSimulator ──▶ PIDController ──▶ ActuatorDriver ──▶ RigidBodySimulator
         ▲                                                           │
         └──────────────────── state feedback ──────────────────────┘
                    │
                    ▼
               CSV log ──▶ Python visualization
```

Every module exposes `checkNumerics()`, all setters assert finiteness, and the manager validates before each step. Non-finite values can't propagate silently.

---

## Visualization Pipeline

`visualization/` reads only the CSV. The components:

`SimulationDataLoader` — loading and sanity checks. The seven kinematic columns are required; the six torque columns are optional and enable the torque overlay when present. Angle magnitudes within $\pi$, rate magnitudes within $10\ \text{rad/s}$, time monotone. Out-of-range values are warnings, not errors.

`AttitudeMathUtils` — three things: the 3-2-1 rotation matrix $R_z(\psi)\,R_y(\theta)\,R_x(\phi)$ in expanded scalar form (three matrix multiplications avoided), radian–degree conversion, and settling detection. The settling band is 2% of the peak excursion from the tail-mean steady state, with an absolute floor. Returns the first index at which the signal permanently enters the band, or −1.

`RigidBodyGeometry` — builds and caches the eight-vertex box mesh and the three body-axis unit vectors, both transformed into the inertial frame.

`AttitudeVisualizer` writes three outputs:

- *Static sequence* — 2×2 figure: isometric, top, and side 3D views of a sampled body sequence along X (inter-body spacing derived from the box half-diagonal, so meshes can't overlap regardless of orientation), plus angle-vs-time with a settling marker
- *Trajectory* — time-colored 3D curve through roll × pitch × yaw space, start/end/settling markers
- *Animation* — one frame per CSV row up to settling plus a margin. The body box and its faint orientation triad carry the attitude; two arrows from the origin, rotated into the inertial frame, carry the dynamics: angular velocity $\boldsymbol\omega$ (magenta) and net control torque $\boldsymbol\tau$ (orange). Telemetry overlay: $\phi/\theta/\psi$, $|\boldsymbol\omega|$, net torque, settling status; time is in the title. The gyroscopic arrow is off by default (`show_gyroscopic_arrow`). Clips ≥ 200 frames on a multi-core host are split across up to 8 worker processes and joined by one `ffmpeg` call; shorter clips and GIF go through `FuncAnimation`. At 100 fps playback is real-time.

`VisualizationConfig` holds all rendering parameters. Entry point: `python -m visualization`.

---

## Reading the Outputs

### Static sequence (`body_orientation_sequence.png`)

A 2×2 figure. The three 3D panels (isometric, top, side) show the same body sequence from different viewpoints; the bottom-right panel is the angle time history.

**Body sequence panels.** Up to 20 body poses are sampled at equal time intervals and laid out left-to-right along the X axis. Color runs viridis from dark-purple (earliest sample) to yellow (latest). Each body carries three short arrows fixed in the body frame: red = X (roll axis), green = Y (pitch axis), blue = Z (yaw axis). Reading the arrow orientations across the sequence shows how the attitude evolves. The isometric view (top-left, elev=20°, azim=45°) gives the clearest spatial picture; the top view (X-Y plane) isolates yaw; the side view (X-Z plane) isolates pitch.

**Angle time history.** Roll, pitch, and yaw in degrees over the full log. Three dashed gray verticals mark the first, middle, and last sampled pose shown in the 3D panels — they link the time axis to the spatial sequence. A solid black vertical marks the detected settling time, if any.

---

### Attitude trajectory (`attitude_trajectory_3d.png`)

A single 3D curve whose axes are roll φ, pitch θ, and yaw ψ in degrees. Each point on the curve is one logged row; the curve is drawn as one segment per step and colored by time using the plasma colormap (dark = early, bright = late). A colorbar on the right gives the time scale.

Three markers: dark-green circle at the start, dark-red square at the end, gold diamond at the settling instant (absent if the run did not settle within the log).

This plot shows the shape of the maneuver in attitude space rather than in time: a tight cluster near a fixed point means the controller has settled; a spread-out curve indicates ongoing transient; sharp corners correspond to setpoint switches.

---

### Animation (`body_animation.mp4`)

One frame per CSV row. At 100 fps, playback runs in real time — one second of video equals one second of simulated time.

The animation runs from $t = 0$ to the settling instant plus a margin (5% of the settling index, minimum 10 frames). If the run does not settle within the log, the full record is animated. The view shows only what is needed to read the controlled motion: the body, its orientation triad, and two dynamics vectors.

**Body and orientation triad.** The translucent box is the rigid body. The three colored axes attached to it (red X / roll, green Y / pitch, blue Z / yaw) rotate with it and are the primary read-out of attitude. The box is drawn translucent and the triad faint so the dynamics arrows below stay legible through them.

**Angular velocity arrow.** The magenta arrow from the origin is $\boldsymbol\omega$ rotated into the inertial frame by $R_z(\psi)\,R_y(\theta)\,R_x(\phi)$, so it shows the instantaneous rotation axis as seen from outside the body. Its length is scaled to the maximum $|\boldsymbol\omega|$ across the animated span — the longest arrow in the video fills the view cube.

**Control torque arrow.** The orange arrow is the net control torque the actuator applies, the action that steers the body toward the setpoint. The raw per-step torque is noise-dominated — the derivative term reacts to sensor noise and chatters by tens of N·m, flipping direction nearly every frame — but that high-frequency part is filtered out by the body's inertia through double integration ($\boldsymbol\tau \to \dot{\boldsymbol\omega} \to \boldsymbol\omega \to \boldsymbol\Theta$) and produces negligible attitude change. The arrow therefore shows the net component (a centered moving average of the torque vector, window `torque_smoothing_window`, default 15 frames), which grows during a maneuver and shrinks toward zero once settled, so it tracks the visible motion. The CSV values are never modified.

The gyroscopic reaction $\boldsymbol\omega \times (I\boldsymbol\omega)$ is logged but not drawn by default — it is an internal term, near-zero except in aggressive slews. Set `show_gyroscopic_arrow` in `VisualizationConfig` to add it (cyan), normalized to its own peak.

**Telemetry overlay** (top-left): attitude angles $\phi/\theta/\psi$, $|\boldsymbol\omega|$, net torque magnitude, and the settling line (`settling...` → `SETTLED   t_s = …`, or `not settled within log`). Time is shown in the title.

---

## Configuration

XML, parsed by `InputParser`. Plain-text line-based format also accepted (used by unit tests).

| Section | Fields | Units |
|---|---|---|
| `ControllerGains` | `Kp`, `Ki`, `Kd` per axis | N·m/rad, N·m/(rad·s), N·m·s/rad |
| `PhysicalProperties` | `Inertia`: $I_{xx}, I_{yy}, I_{zz}$ | kg·m² |
| `SensorCharacteristics` | `DriftRate`, `NoiseStdDev` per axis | rad/s, rad |
| `ActuatorProperties` | `Delay`, `MaxTorquePerAxis`, `MaxTorqueMagnitude` | s, N·m, N·m |
| `ControllerParameters` | `Smoothing` ($\alpha \in [0,1]$), `AntiWindup` ($\tau$, s), `RateFeedback` (0 or 1) | — |
| `InitialConditions` | `Attitude` (roll, pitch, yaw), `AngularVelocity` | rad, rad/s |
| `SetpointSequence` | timestamped `Setpoint` entries, `time` strictly increasing | s, rad |

Six reference configs ship in pairs — same plant, gains, sensors and maneuver profile; only the derivative path differs:

| Attitude derivative (`RateFeedback=0`) | Measured rate (`RateFeedback=1`) |
|---|---|
| `config_best_case.xml` | `config_best_case_rate_feedback.xml` |
| `config_normal_case.xml` | `config_normal_case_rate_feedback.xml` |
| `config_worst_case.xml` | `config_worst_case_rate_feedback.xml` |

**Best** — near-ideal sensing ($\sigma = 0.8\ \text{mrad/axis}$, zero drift), 5 ms delay, inertia $[15, 14, 16]\ \mathrm{kg\,m^2}$, small-amplitude slews ($\omega_n \approx 1.5\ \text{rad/s}$, $\zeta \approx 0.9$). Five waypoints to $t = 20\ \text{s}$.

**Normal** — representative sensing ($\sigma = 3\ \text{mrad/axis}$, zero drift), 20 ms delay, inertia $[20, 18, 24]\ \mathrm{kg\,m^2}$, mission profile with return to nadir ($\omega_n \approx 1.2\ \text{rad/s}$, $\zeta \approx 0.85$). Five waypoints to $t = 30\ \text{s}$.

**Worst** — aggressive sensing ($\sigma = 6\ \text{mrad/axis}$, $3\times10^{-5}\ \text{rad/s}$ drift), 40 ms delay, tighter torque limits, inertia $[25, 20, 30]\ \mathrm{kg\,m^2}$, three sign-reversing slews ($\omega_n \approx 1.0\ \text{rad/s}$, $\zeta \approx 0.8$). Eight waypoints to $t = 58\ \text{s}$.

---

## Results

Each pair was integrated at $\Delta t = 0.01\ \text{s}$ (100 Hz) for $t = 200\ \text{s}$. The maneuver finishes in the first 20–58 s; the remaining 140–180 s are a hold at the origin so steady-state pointing is measured on a relaxed loop. Runs are deterministic for the fixed RNG seed.

**Metrics.** Per-axis 2% settling time $t_s$: time from a step until the axis stays within $\max(0.02\,\Delta\theta,\ 0.3°)$ of the new target. Overshoot: peak excursion past the target as a fraction of the step amplitude. Steady-state error: 3-axis RMS true attitude over the final 2 s. Peak torque, peak body rate and peak gyroscopic torque over the full log.

### Attitude derivative vs measured rate

| | Best (att.) | Best (rate) | Normal (att.) | Normal (rate) | Worst (att.) | Worst (rate) |
|---|---|---|---|---|---|---|
| Settling $t_s$ mean / max | 3.20 / 3.62 s | 3.45 / 4.05 s | 3.93 / 4.96 s | 4.23 / 5.73 s | 6.20 / 8.58 s | 4.81 / 7.09 s |
| Overshoot mean / max | 0.0 / 0.1 % | 2.5 / 10.8 % | 0.4 / 0.9 % | 3.3 / 17.1 % | 3.5 / 12.2 % | 1.4 / 7.3 % |
| Hold error — 3-axis RMS | 0.008° | **0.003°** | 0.12° | **0.010°** | **0.75°** | 0.87° |
| Peak $\lvert\boldsymbol\tau\rvert$ | 38.0 N·m | 15.7 N·m | 34.0 N·m | 20.7 N·m | 32.0 N·m | 20.0 N·m |
| Peak $\lvert\boldsymbol\omega\rvert$ | 16.5 °/s | 15.2 °/s | 16.0 °/s | 17.2 °/s | 20.3 °/s | 22.1 °/s |
| Peak gyroscopic torque | 0.071 N·m | 0.065 N·m | 0.170 N·m | 0.190 N·m | 0.395 N·m | 0.274 N·m |

**Best.** Rate feedback sharpens the long hold (0.008° → 0.003° RMS) by replacing the differentiated attitude error with $\tilde\omega$. Transient overshoot rises because $K_d\tilde\omega$ reacts to measurement noise on the gyro channel; peak torque drops well below the 38 N·m cap because the D-term no longer amplifies attitude noise.

**Normal.** Same trade-off at operational noise levels: hold error falls from 0.12° to 0.01° RMS — an order of magnitude — at the cost of higher overshoot on the larger slews. Settling stretches by $\approx 0.3\ \text{s}$ mean.

**Worst.** Rate feedback shortens settling (6.2 → 4.8 s mean) and cuts overshoot (12% → 7% max) on the aggressive reversals, but **does not** fix the drift-induced hold offset: hold error worsens slightly (0.75° → 0.87° RMS). The sensor model ramps both $\tilde\theta$ and $\tilde\omega$ with the same $\dot b$, so $-K_d\tilde\omega$ injects a biased D-term while the type-1 loop still cannot reject the attitude ramp. One closing pitch step remains outside the 2% band with attitude derivative; it settles with rate feedback.

The peak-torque drop in the rate-feedback runs (38 → 16 N·m best, 34 → 21 N·m normal, 32 → 20 N·m worst) shows that a large fraction of the attitude-derivative command was high-frequency noise gain, not net maneuver torque.

---

## Build and Usage

### C++ simulator

Requires `tinyxml`. The Makefile probes once at first build, stamps a file, and can install via Homebrew or the usual Linux package managers. `make check-tinyxml` runs the probe alone.

```bash
make                  # attitude_simulator binary
make release          # optimized, asserts become ErrorCode returns
make all-tests        # simulator + test executables
make re               # full rebuild
make clean            # objects + Python bytecode
make fclean           # clean + executables + simulation output
```

Bare filenames resolve under `simulation_input/`. Defaults: duration 10 s, timestep 0.01 s.

```bash
./attitude_simulator <config.xml> [duration] [timestep]
./attitude_simulator config_normal_case.xml 100.0 0.01
```

Output: `simulation_output/simulation_output.csv`.

### Python visualization

```bash
make venv                                           # .venv/ + requirements.txt
python -m visualization                             # interactive menu
make test-visualization                             # unittest suite
make test-visualization PYTHON=./.venv/bin/python   # specific interpreter
```

`ffmpeg` on `PATH` for MP4. Defaults to `Agg`; override with `MPLBACKEND`.

---

## Project Structure

```
attitude_controller_simulator/
│
├── main.cpp
├── Makefile
├── requirements.txt                 # pandas, numpy, matplotlib, pillow
│
├── includes/
│   ├── control/   (PID, PIDController, ActuatorDriver)
│   ├── io/        (InputParser)
│   ├── manager/   (SimulationManager, ErrorCodes)
│   ├── physics/   (RigidBodySimulator, Vector3f)
│   └── sensor/    (SensorSimulator, GaussianNoise)
├── src/
│
├── visualization/
│   ├── AttitudeAnalysisApplication.py
│   ├── AttitudeVisualizer.py
│   ├── SimulationDataLoader.py
│   ├── AttitudeMathUtils.py
│   ├── RigidBodyGeometry.py
│   └── VisualizationConfig.py
│
├── simulation_input/
├── simulation_output/
│
└── tests/
    ├── control/    test_pid_controller.cpp
    ├── gtest/      test_gtest.cpp         (C++17)
    ├── io/         test_input_parser.cpp
    ├── manager/    test_simulation_manager.cpp
    ├── test_pid_rbs_vec3f.cpp
    └── visualization/
```

---

## Testing

`make all-tests` builds five C++ executables plus `test_gtest` (C++17). Each suite targets one layer of the stack with explicit, checkable contracts.

**`test_pid_controller`** — discrete PID numerics: proportional step response, integral clamp at $\pm 100$, EMA derivative attenuation, output saturation at $\pm 1000$, three-axis decoupling. Rate-feedback path: verifies $\tau = K_p e - K_d \omega$ when `computeWithBodyRate` is called with $\omega = 0$ (must match `compute` on the first step), and that a non-zero $\omega$ replaces the finite-difference derivative.

**`test_pid_rbs_vec3f`** — `RigidBodySimulator` integration against hand-checked torques; `PIDController` wired to the plant without sensors.

**`test_input_parser`** — XML/TXT loading, setpoint interpolation, `reset`, copy semantics. Reference configs: `RateFeedback=0` on `config_*_case.xml`, `RateFeedback=1` on `config_*_case_rate_feedback.xml`; missing tag defaults to disabled.

**`test_simulation_manager`** — configuration load, safety-limit validation, state-machine guards, and a 0.1 s closed-loop run for both `config_normal_case.xml` and `config_normal_case_rate_feedback.xml` (must reach `STATE_COMPLETED` without `ERR_RT_NAN_DETECTED`).

**`test_gtest`** — GTest coverage of sensor drift integration, actuator delay FIFO, parser error codes, and the rate-feedback torque law on a single axis.

**`make test-visualization`** — Python `unittest` on the post-processing layer: CSV column contract vs `SimulationManager::exportLog`, rotation-matrix orthonormality and 3-2-1 product, box-mesh geometry, PNG/MP4 export, and settling-index detection on a synthetic decaying signal.

---

## Design Scope

The choices below are deliberate simplifications that keep the implementation tractable and focused on what the testbed is built to study: closed-loop attitude control behavior under realistic sensor and actuator constraints.

**Fixed-step semi-implicit Euler.** The integrator step matches the controller's sampling period by construction — the two are not decoupled, so the discrete-time closed-loop behavior is directly interpretable without re-discretizing a continuous plant. For the maneuver scales this testbed targets, the local truncation error is well below the sensor noise floor, so a higher-order integrator would add complexity without changing the observable dynamics. Semi-implicit Euler also conserves the skew-symmetric structure of the gyroscopic term better than explicit Euler, at zero additional cost.

**Diagonal inertia.** The principal-axis assumption is the standard starting point for attitude controller design. It isolates the gyroscopic coupling already present in the diagonal case — which is the dominant cross-axis effect — while keeping the inertia parameterization minimal. Off-diagonal terms are introduced in a later refinement stage in real programs, once the baseline controller is validated; including them here would obscure the coupling the model is specifically designed to expose.

**Decoupled single-axis PID.** Three independent PID controllers is the architecture actually flown on the majority of operational attitude control systems. Decoupling makes the stability analysis tractable: Bode and root-locus tools apply to each axis independently, and the gain-tuning problem decomposes. The gyroscopic coupling is present in the plant model, so its effect on the transient response is fully observable in the output even though the controller doesn't explicitly compensate for it — which is the pedagogically useful case.

**Euler-angle kinematics.** Addressed in §Attitude Kinematics above.

C++98 / MISRA C++:2008 was a deliberate choice, not a legacy constraint. The restricted language subset eliminates entire classes of undefined behavior — dynamic allocation, exceptions, complex template metaprogramming — and makes every code path statically bounded and manually reviewable. Compliance is checked manually against the standard; no certified static analysis tool (e.g. PC-lint, Polyspace, PRQA QA·C++) was used, so the codebase does not constitute a certifiable artifact under any safety standard. Google Test is the only C++17 dependency; it doesn't touch any production code path. Debug builds use assertions; `make release` replaces them with `ErrorCode` returns, maintaining the same observable behavior at higher optimization.

---

## Author and License

Written by Antonio Pintauro ([@denuen](https://github.com/denuen)). Personal project, built for learning. Each modelling choice in §Design Scope reflects the testbed's intended scope; validating these models against certified tools or real hardware would be a separate, differently-scoped effort.

MIT License — see [`LICENSE`](LICENSE). No warranty, no liability.
