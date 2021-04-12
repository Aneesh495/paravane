# Paravane

```
  ____                                               
 |  _ \ __ _ _ __ __ ___   ____ _ _ __   ___         
 | |_) / _` | '__/ _` \ \ / / _` | '_ \ / _ \        
 |  __/ (_| | | | (_| |\ V / (_| | | | |  __/        
 |_|   \__,_|_|  \__,_| \_/ \__,_|_| |_|\___|        
                                                     
 Autonomous Underwater Vehicle Navigation & SLAM Engine
```

[![Language: Janet](https://img.shields.io/badge/Language-Janet%201.37+-blue.svg)](https://janet-lang.org/)
[![Acceleration: C11](https://img.shields.io/badge/Acceleration-C11%20Clang-green.svg)]()
[![Tests: 100% Passing](https://img.shields.io/badge/Tests-10%20Suites%20Passing-brightgreen.svg)]()
[![License: MIT](https://img.shields.io/badge/License-MIT-purple.svg)](LICENSE)

**Paravane** is an autonomous underwater vehicle (AUV) mission executive, acoustic navigation engine, sound velocity ray tracer, and bathymetric mapping platform. Designed for deep ocean exploration, continental shelf surveys, and hydrothermal vent mapping, Paravane combines the expressive functional scripting of **Janet** with native C acceleration routines for high-throughput linear algebra, Runge-Kutta numerical ray shooting, and zero-allocation subsea telemetry codecs.

---

## Architecture Overview

```
+--------------------------------------------------------------------------+
|                     Executive & Autonomy Layer                           |
|     Behavior Trees (bt)  |  Fiber Scheduler (fe)  |  FDIR Supervisor     |
+--------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------+
|                      Guidance & Survey Planning                          |
|   3D Line-of-Sight (los) | Dubins 3D Curves | Boustrophedon Survey Grid  |
+--------------------------------------------------------------------------+
          |                                               |
          v                                               v
+---------------------------------------+   +------------------------------+
|     Navigation & State Estimation     |   |      Bathymetric SLAM        |
|  15-State ES-EKF  | Janus 4-Beam DVL  |   | 2.5D DEM Grid | Multibeam MBES|
|  Strapdown INS    | Depth Transducer  |   | Terrain Matching | Arun ICP  |
+---------------------------------------+   +------------------------------+
                    |                                     |
                    +------------------+------------------+
                                       |
                                       v
+--------------------------------------------------------------------------+
|                       Acoustics & Ocean Physics                          |
|  Hamiltonian Ray Tracer (RK4) | USBL / LBL Inversion | Thorp Attenuation |
|  Munk Sound Speed Duct        | UNESCO IES-80 Density | Ekman Spiral     |
+--------------------------------------------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------+
|                        Core Mathematical Engine                          |
|  WGS-84 Geodesy | Quaternions | 15x15 Matrix Math | Welford Statistics   |
+--------------------------------------------------------------------------+
                                       |
                                       v
+--------------------------------------------------------------------------+
|                   C Native Acceleration (libparavane)                    |
|  RK4 Ray Propagation | SIMD Covariance Math | Point Cloud Nearest Neighbor|
+--------------------------------------------------------------------------+
```

---

## Core Capabilities

### 1. Stratified Ocean Physics & Acoustics
- **Equations of State**: Computes thermodynamic seawater density using UNESCO IES-80 and converts hydrostatic pressure to meters depth using the Saunders (1981) latitude-dependent gravity formula.
- **Sound Velocity Profiles**: Implements the 9-term Mackenzie equation, UNESCO Chen-Millero (1977) with Wong-Zhu (1995) corrections, and canonical Munk SOFAR sound channel ducting models.
- **Hamiltonian 4th-Order Runge-Kutta Ray Tracer**: Propagates acoustic rays through continuous cubic spline sound speed profiles parameterized by arc length $s$, accurately modeling refractive bending, turning points, caustic formations, and specular sea-surface and seabed reflections.
- **Transmission Loss & Sonar Budget**: Evaluates geometric spreading loss, Francois-Garrison and Thorp chemical absorption (boric acid and magnesium sulfate relaxation), and Wenz deep-sea ambient noise models.
- **Acoustic Positioning**: Solves Phase Difference of Arrival (PDOA) Ultra-Short Baseline (USBL) slant range and bearing inversion with vessel attitude compensation, alongside Long Baseline (LBL) Gauss-Newton multilateration with GDOP, HDOP, and VDOP metrics.

### 2. Subsea Inertial Navigation & Sensor Fusion
- **Strapdown INS Mechanization**: 100 Hz kinematic quaternion integration in the local North-East-Down (NED) coordinate frame, accounting for Coriolis acceleration and local gravity anomalies.
- **15-State Error-State Kalman Filter (ES-EKF)**: Fuses high-rate tactical IMU specific force and angular rate with asynchronous updates:
  - Doppler Velocity Log (DVL): 4-beam Janus configuration with bottom-track lock detection and sound speed scaling.
  - Quartz Bourdon Depth Transducer: Hydrostatic pressure observation with chi-square innovation gating.
  - Periodic USBL / LBL Acoustic Fixes: Direct error-state injection and covariance reset.
- **Allan Variance Noise Models**: Accurately tracks accelerometer and rate gyro random walk biases.

### 3. 6-DOF Hydrodynamics & Propulsion
- **Fossen Marine Craft Kinetics**: Evaluates the non-linear equations of motion:
  $$\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}$$
- **Hydrodynamic Added Mass**: Models diagonal and coupled hydrodynamic inertia ($X_{\dot{u}}, Y_{\dot{v}}, Z_{\dot{w}}, K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}}$) for survey-class AUVs (REMUS 100 / HUGIN 1000).
- **Restoring Forces & Metacentric Stability**: Computes hydrostatic righting moments separating center of gravity (CG) and center of buoyancy (CB).
- **Thruster Allocation Matrix (TAM)**: Solves optimal multi-thruster force distribution via Moore-Penrose pseudo-inverse with quadratic energy minimization and thrust saturation limits.

### 4. Guidance & Survey Mission Planning
- **3D Line-of-Sight (LOS) Guidance**: Computes cross-track steering laws with dynamic lookahead distance and ocean current sideslip drift angle compensation.
- **Dubins 3D Path Planning**: Generates minimum-time, continuous-curvature trajectories between waypoints with pitch rate and yaw rate turning radius constraints.
- **Boustrophedon Survey Grids**: Plans optimal lawnmower tracklines with multibeam swath overlap control and orthogonal cross-tie check lines for bathymetric calibration.
- **Bottom-Following & Obstacle Avoidance**: Maintains fixed altitude above undulating seafloors and evaluates Forward-Looking Sonar (FLS) acoustic sectors for reactive obstacle avoidance.

### 5. Bathymetric SLAM & Mapping
- **Multibeam Echo-Sounder (MBES)**: Ray-traces individual acoustic swath beam angles, correcting for pitch, roll, yaw, and acoustic sound velocity refraction.
- **2.5D Digital Elevation Model (DEM) Grid**: Employs spatial hashing with online Welford sample mean and variance tracking for every seafloor grid cell.
- **Terrain-Aided Navigation (TAN)**: Implements non-parametric particle filtering matching multibeam depth observations against pre-existing bathymetric charts.
- **Point Cloud Registration**: Executes Arun Singular Value Decomposition (SVD) and Iterative Closest Point (ICP) algorithms to align overlapping survey swaths.

### 6. Acoustic Telemetry & Executive Architecture
- **JANUS IEEE 1841 Codec**: Encodes and decodes standard 80-bit subsea acoustic modem packets with hardware-grade zero-allocation CCITT CRC-16 checksums.
- **NMEA-0183 Telemetry Parser**: High-performance Parsing Expression Grammar (PEG) extraction for `$PASHR` (inertial attitude), `$PD4` (DVL velocity), and `$DBT` (depth below transducer).
- **RFC 7946 GeoJSON Exporter**: Converts local NED mission trajectories and survey waypoints to geodetic WGS-84 GeoJSON FeatureCollections for GIS workflows.
- **Cooperative Fiber Scheduler**: Deterministic, single-threaded cooperative fiber scheduler and hierarchical Behavior Tree engine executing mission state machines.
- **FDIR Safety Executive**: Ray-casting polygonal geofencing, leak detection, battery management, and emergency positive buoyancy ballast drop.

---

## Project Structure

```
paravane/
├── c/                               # Native C11 acceleration routines
│   ├── paravane_native.h            # C API definitions for ray tracing, matrix math & CRC
│   └── paravane_native.c            # SIMD matrix math & Runge-Kutta numerical kernels
├── docs/                            # In-depth technical documentation
│   ├── architecture.md              # System architecture & cooperative fiber engine
│   ├── acoustics.md                 # Mathematical derivations of ray tracing & sonar
│   ├── navigation.md                # 15-state ES-EKF formulation & covariance mechanics
│   └── hydrodynamics.md             # 6-DOF Fossen kinematics & thruster allocation
├── examples/                        # Standalone mission demonstrations
│   └── survey_deep_ridge.janet      # Mid-Atlantic fracture zone survey simulation
├── src/paravane/                    # Janet core subsea robotics framework
│   ├── init.janet                   # Master module exports
│   ├── main.janet                   # Unified CLI entrypoint with subcommands
│   ├── core/                        # Mathematical and geometrical primitives
│   │   ├── constants.janet          # Physical constants & WGS-84 ellipsoid parameters
│   │   ├── math.janet               # 3D vectors, 3x3 matrices, quaternions, Euler angles
│   │   ├── geodesy.janet            # WGS-84, ECEF, and local tangent NED coordinates
│   │   └── stats.janet              # Welford online mean & variance accumulation
│   ├── ocean/                       # Physical oceanography & seawater thermodynamics
│   │   ├── sound_speed.janet        # Mackenzie, Chen-Millero, Del Grosso, Coppens formulas
│   │   ├── equation_of_state.janet  # UNESCO IES-80 density & Saunders pressure-to-depth
│   │   ├── profile.janet            # Cubic spline SVP interpolation & Munk duct
│   │   └── currents.janet           # Ekman spiral & M2 tidal current models
│   ├── acoustics/                   # Underwater acoustics & positioning
│   │   ├── ray_tracer.janet         # Hamiltonian 4th-order Runge-Kutta acoustic ray tracer
│   │   ├── transmission_loss.janet  # Thorp attenuation, spherical spreading, Wenz noise
│   │   ├── usbl.janet               # Ultra-Short Baseline PDOA & attitude compensation
│   │   └── lbl.janet                # Long Baseline Gauss-Newton multilateration & GDOP
│   ├── navigation/                  # Inertial navigation & state estimation
│   │   ├── sensors.janet            # Tactical IMU, DVL, and depth sensor models
│   │   ├── dvl.janet                # Janus 4-beam geometry & 3-beam fallback
│   │   ├── strapdown.janet          # 100 Hz quaternion strapdown INS mechanization
│   │   └── ekf.janet                # 15-state Error-State Extended Kalman Filter
│   ├── hydrodynamics/               # 6-DOF marine kinetics & propulsion
│   │   ├── fossen.janet             # 6-DOF marine craft equations of motion
│   │   ├── vehicle_model.janet      # REMUS/HUGIN survey class vehicle parameters
│   │   ├── buoyancy_engine.janet    # Variable buoyancy system & sliding mass pitch trim
│   │   └── thrusters.janet          # Thruster Allocation Matrix pseudo-inverse solver
│   ├── guidance/                    # Trajectory generation & path following
│   │   ├── los.janet                # 3D Line-of-Sight guidance & drift compensation
│   │   ├── dubins.janet             # 3D continuous-curvature Dubins curves
│   │   ├── survey.janet             # Boustrophedon lawnmower survey grid planner
│   │   ├── terrain_following.janet  # Bottom-following altitude hold with flare recovery
│   │   └── obstacle_avoidance.janet # Forward-Looking Sonar sector avoidance
│   ├── slam/                        # Bathymetric mapping & localization
│   │   ├── mbes.janet               # Multibeam swath projection & beam refraction
│   │   ├── bathymetry_grid.janet    # 2.5D DEM grid with Welford variance tracking
│   │   ├── terrain_matching.janet   # Terrain-Aided Navigation particle filter
│   │   └── point_cloud.janet        # 3D point cloud nearest neighbor & Arun ICP
│   ├── comms/                       # Subsea acoustic communication protocols
│   │   └── janus.janet              # JANUS IEEE 1841 80-bit acoustic packet codec
│   ├── telemetry/                   # Subsea telemetry parsers & GIS exporters
│   │   ├── nmea.janet               # High-speed PEG parsers for $PASHR, $PD4, $DBT
│   │   └── geojson.janet            # RFC 7946 GeoJSON trajectory exporter
│   └── executive/                   # Mission autonomy & fault management
│       ├── fiber_engine.janet       # Deterministic cooperative fiber task scheduler
│       ├── behavior_tree.janet      # Hierarchical Behavior Tree execution engine
│       └── fault_manager.janet      # FDIR executive & polygonal geofencing
├── test/                            # Comprehensive verification test suites
│   ├── helpers.janet                # Test harness assertions & statistical reporters
│   ├── runner.janet                 # Master test orchestrator
│   ├── test_native.c                # C acceleration verification suite
│   ├── test_core_math.janet         # 40 assertions (WGS-84, quaternions, matrices)
│   ├── test_ocean_physics.janet     # 28 assertions (IES-80, Saunders, Chen-Millero)
│   ├── test_acoustics.janet         # 20 assertions (RK4 ray tracing, USBL, LBL)
│   ├── test_navigation.janet        # 15 assertions (INS strapdown, DVL, ES-EKF)
│   ├── test_hydrodynamics.janet     # 20 assertions (Fossen kinetics, TAM solver)
│   ├── test_guidance.janet          # 21 assertions (3D LOS, Dubins, Boustrophedon)
│   ├── test_slam.janet              # 20 assertions (MBES, 2.5D DEM, Arun ICP)
│   ├── test_comms_telemetry.janet   # 21 assertions (JANUS packets, NMEA, GeoJSON)
│   ├── test_executive.janet         # 12 assertions (fibers, behavior trees, FDIR)
│   └── test_e2e_mission.janet       # 9 assertions (closed-loop mission simulation)
├── tools/                           # Python simulation & visualization utilities
│   ├── bathymetry_generator.py      # Fractal seabed DEM generator with seamounts
│   ├── simulator.py                 # Synthetic 6-DOF ocean & sensor stream generator
│   ├── visualizer.py                # Standalone HTML5/SVG interactive dashboard
│   └── check_style.py               # Repository style & character invariant checker
├── Makefile                         # Unified compilation & verification runner
└── project.janet                    # JPM package declaration
```

---

## Getting Started

### Prerequisites
- **Janet** (>= 1.34.0, 1.37.0+ recommended)
- **Clang** or **GCC** (C11 support for native acceleration)
- **Python 3** (standard library only, for simulation and visualization tools)
- **Make**

### Build and Verify

Clone the repository and run the test suite:

```bash
git clone https://github.com/Aneesh495/paravane.git
cd paravane

# Verify Janet syntax and compile C acceleration library
make build

# Execute all unit, structure, integration, and native test suites
make test

# Verify repository hygiene and character invariants
make check-style
```

### CLI Subcommands

Paravane provides a unified command-line entrypoint (`src/paravane/main.janet`):

```bash
# Display subsystem diagnostics and environment readiness
janet src/paravane/main.janet status

# Generate a Boustrophedon lawnmower survey grid: origin (0,0), 400x200m, 50m lane spacing, 1150m depth
janet src/paravane/main.janet survey 0 0 400 200 50 1150

# Trace acoustic ray: source depth 1000m, launch angle 0.05 rad (2.86 deg), range 3000m
janet src/paravane/main.janet ray-trace 1000 0.05 3000

# Parse NMEA-0183 subsea telegram ($PASHR attitude)
janet src/paravane/main.janet parse-nmea "$PASHR,123456.78,142.50,T,1.25,-2.10,0.05,0.02,0.02,0.05,1,1*23"

# Run high-performance micro-benchmarks
janet src/paravane/main.janet benchmark

# Execute the complete autonomous subsea survey mission
janet src/paravane/main.janet run-mission examples/survey_deep_ridge.janet
```

### Simulation and Visualization Tools

Paravane includes standalone visualization and simulation scripts that run with zero external pip dependencies:

```bash
# 1. Synthesize a fractal seabed bathymetry field with seamounts and hydrothermal vents
python3 tools/bathymetry_generator.py --width 120 --height 120 --resolution 5.0 --out bathymetry.json

# 2. Simulate a 6-DOF subsea mission with streaming IMU, DVL, depth, and USBL telemetry
python3 tools/simulator.py --duration 120.0 --dt 0.1 --out telemetry.json

# 3. Generate a self-contained interactive HTML5 / SVG mission dashboard
python3 tools/visualizer.py --telemetry telemetry.json --bathymetry bathymetry.json --out mission_dashboard.html
```

Open `mission_dashboard.html` in any modern web browser to view the horizontal survey track, depth profile, acoustic ray fan through the SOFAR channel, and bathymetric DEM heatmap.

---

## Verification & Test Suite Summary

Paravane maintains 100% test pass rates across 10 specialized test suites and 219 total assertions:

| Test Suite | Scope & Key Invariants Verified | Assertions | Result |
| :--- | :--- | :---: | :---: |
| `test_native.c` | C11 SIMD 15x15 matrix algebra, Runge-Kutta 4th-order ray tracer, CCITT CRC-16 | 13 | **PASS** |
| `test_core_math.janet` | WGS-84 geodesy, quaternion rotation, skew-symmetric matrices, Welford statistics | 40 | **PASS** |
| `test_ocean_physics.janet` | UNESCO IES-80 density, Saunders (1981) depth, Chen-Millero sound speed, Munk duct | 28 | **PASS** |
| `test_acoustics.janet` | Hamiltonian RK4 ray tracing, Snell invariant, Thorp absorption, USBL/LBL solvers | 20 | **PASS** |
| `test_navigation.janet` | Strapdown INS mechanization, Janus 4-beam DVL transforms, 15-state ES-EKF updates | 15 | **PASS** |
| `test_hydrodynamics.janet` | Fossen 6-DOF kinetics, REMUS parameters, restoring moments, TAM pseudo-inverse | 20 | **PASS** |
| `test_guidance.janet` | 3D LOS drift compensation, Dubins 3D curves, Boustrophedon grid, FLS avoidance | 21 | **PASS** |
| `test_slam.janet` | Multibeam swath refraction, 2.5D DEM grid variance tracking, Arun ICP registration | 20 | **PASS** |
| `test_comms_telemetry.janet` | JANUS IEEE 1841 80-bit codec, NMEA PEG parsers, RFC 7946 GeoJSON export | 21 | **PASS** |
| `test_executive.janet` | Cooperative fiber scheduler, hierarchical Behavior Trees, FDIR polygonal geofence | 12 | **PASS** |
| `test_e2e_mission.janet` | End-to-end mission lifecycle: descent, bottom-lock, survey grid, mapping, ascent | 9 | **PASS** |
| **Total** | **Comprehensive Full-System Verification Coverage** | **219** | **100%** |

---

## Implemented Oceanographic Models

- **Seawater Density**: UNESCO IES-80 international equation of state formulation.
- **Hydrostatic Depth**: Saunders (1981) latitude-dependent pressure-to-depth polynomial.
- **Sound Velocity**: UNESCO Chen-Millero, Mackenzie 9-term, and canonical Munk SOFAR duct profiles.
- **Subsea Kinematics**: SNAME notation and 6-DOF marine craft dynamics.
- **Chemical Attenuation**: Thorp and Francois-Garrison low-frequency absorption equations.
- **Online Variance**: Welford single-pass statistical accumulation.
- **Acoustic Packets**: JANUS IEEE 1841 telemetry framing and CCITT CRC-16 checksums.

---

## License

Paravane is released under the **MIT License**. See [LICENSE](LICENSE) for details.
