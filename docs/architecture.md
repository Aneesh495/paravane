# Paravane System Architecture

Paravane is an autonomous underwater vehicle (AUV) mission executive, acoustic navigation engine, sound velocity ray tracer, and bathymetric mapping platform written primarily in Janet, with native C acceleration routines for high-throughput linear algebra and numerical ray shooting.

## Subsystem Hierarchy

The software architecture is structured into decoupled, high-cohesion layers:

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

## Cooperative Fiber Concurrency

Subsea embedded computers operating deep ocean robotics must adhere to strict deterministic execution budgets without the latency jitter or memory synchronization overhead of OS-level preemption threads.

Paravane implements a lightweight cooperative fiber scheduler (`src/paravane/executive/fiber_engine.janet`) that multiplexes critical subsea control loops onto a single OS thread:

1. **Navigation Fusion Task (100 Hz / 10 Hz)**: Mechanizes high-rate strapdown inertial integration, accumulates accelerometer/gyro bias random walks, and fuses Doppler Velocity Log (DVL) bottom-track velocity.
2. **Multibeam Acoustic Acquisition Task (2 Hz)**: Triggers multibeam sonar pings across the swath, computes geometric ray raycasts against the sound speed profile, and grids depth soundings into the 2.5D Digital Elevation Model.
3. **Guidance and Path Control Task (1 Hz)**: Solves 3D Line-of-Sight (LOS) cross-track error kinematics, commands yaw and pitch rates, and advances survey waypoints.
4. **FDIR Safety Monitor Task (5 Hz)**: Evaluates hull humidity, battery state-of-charge, geofence polygons, and acoustic silence timers, triggering emergency positive buoyancy if safety invariants are breached.

## Inter-Subsystem Communication

Subsystems communicate through structured immutable messages and a centralized blackboard table:
- **Telemetry Ingestion**: Telemetry logs and serial strings are parsed via high-speed Parsing Expression Grammars (PEG) for NMEA-0183 sentences ($PASHR attitude, $PD4 DVL velocity, $DBT depth).
- **Acoustic Comms Codec**: JANUS IEEE 1841 compliant 80-bit acoustic telemetry packets are packed and unpacked with zero heap allocations, protected by CCITT CRC-16 checksums.
- **Data Export**: Mission tracks and surveyed seafloor features are exported to RFC 7946 GeoJSON and standard JSON telemetry streams for real-time visualization in browser dashboards.
