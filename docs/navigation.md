# Subsea Inertial Navigation & 15-State Error-State EKF

Subsea vehicles operate in GPS-denied environments. Accurate long-endurance dead reckoning requires high-rate strapdown inertial navigation tightly coupled with acoustic bottom-tracking Doppler Velocity Logs (DVL), piezoresistive depth sensors, and periodic acoustic positioning updates.

## 15-State Error-State Kalman Filter (ES-EKF)

Paravane implements an Error-State Extended Kalman Filter (also termed indirect Kalman filter). The filter partitions state estimation into:
1. **Nominal State $\mathbf{x}$**: High-rate, non-linear inertial mechanization running at 100 Hz.
2. **Error State $\delta \mathbf{x}$**: True state minus nominal state, operating near zero, allowing linearized error dynamics.

The 15-dimensional error state vector is defined as:

$$\delta \mathbf{x} = \begin{bmatrix} \delta \mathbf{p}^n \\ \delta \mathbf{v}^n \\ \delta \boldsymbol{\theta}^n \\ \delta \mathbf{b}_a \\ \delta \mathbf{b}_g \end{bmatrix} \in \mathbb{R}^{15}$$

where:
- $\delta \mathbf{p}^n \in \mathbb{R}^3$: Position error in North-East-Down (NED) frame.
- $\delta \mathbf{v}^n \in \mathbb{R}^3$: Velocity error in NED frame.
- $\delta \boldsymbol{\theta}^n \in \mathbb{R}^3$: Small-angle orientation attitude error in NED frame.
- $\delta \mathbf{b}_a \in \mathbb{R}^3$: Accelerometer sensor bias drift in body frame.
- $\delta \mathbf{b}_g \in \mathbb{R}^3$: Gyroscope sensor bias drift in body frame.

## Continuous-Time Error Dynamics

The linearized continuous-time error state dynamics follow:

$$\dot{\delta \mathbf{x}}(t) = \mathbf{F}(t) \delta \mathbf{x}(t) + \mathbf{G}(t) \mathbf{w}(t)$$

where the system transition matrix $\mathbf{F} \in \mathbb{R}^{15 \times 15}$ is:

$$\mathbf{F} = \begin{bmatrix}
\mathbf{0}_{3 \times 3} & \mathbf{I}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & -[\mathbf{f}^n]_\times & -\mathbf{R}_b^n & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & -\mathbf{R}_b^n \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \\
\mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3}
\end{bmatrix}$$

Here:
- $\mathbf{R}_b^n$ is the rotation matrix transforming from vehicle body frame to NED frame.
- $\mathbf{f}^n = \mathbf{R}_b^n (\mathbf{f}^b - \mathbf{b}_a)$ is specific force in NED coordinates.
- $[\mathbf{f}^n]_\times$ is the $3 \times 3$ skew-symmetric matrix representing cross-product operations.

## Discrete Time Propagation

For a discrete integration time step $\Delta t$, the state transition matrix is approximated as:

$$\boldsymbol{\Phi}_k = \mathbf{I}_{15 \times 15} + \mathbf{F}_k \Delta t$$

The error covariance matrix $\mathbf{P} \in \mathbb{R}^{15 \times 15}$ propagates as:

$$\mathbf{P}_{k|k-1} = \boldsymbol{\Phi}_k \mathbf{P}_{k-1|k-1} \boldsymbol{\Phi}_k^T + \mathbf{Q}_k \Delta t$$

where $\mathbf{Q}_k = \text{diag}(\sigma_p^2, \sigma_v^2, \sigma_\theta^2, \sigma_{ba}^2, \sigma_{bg}^2)$ models white sensor noise and random walk drift.

## Measurement Updates

### 1. Doppler Velocity Log (DVL)
The DVL measures 3D velocity in the vehicle body frame $\mathbf{v}^b$:

$$\mathbf{z}_{\text{DVL}} = \mathbf{R}_n^b \mathbf{v}^n + \mathbf{v}_{\text{noise}}$$

The observation matrix is:

$$\mathbf{H}_{\text{DVL}} = \begin{bmatrix} \mathbf{0}_{3 \times 3} & \mathbf{R}_n^b & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} & \mathbf{0}_{3 \times 3} \end{bmatrix} \in \mathbb{R}^{3 \times 15}$$

### 2. Quartz Pressure Depth Transducer
The depth transducer measures hydrostatic pressure, yielding scalar depth $z_{\text{depth}} = p_{\text{down}}$:

$$\mathbf{H}_{\text{depth}} = \begin{bmatrix} 0 & 0 & 1 & \mathbf{0}_{1 \times 12} \end{bmatrix} \in \mathbb{R}^{1 \times 15}$$

### 3. Absolute Acoustic Position Fix (USBL / LBL)
When acoustic transponder fixes are received from surface ship or seafloor beacons:

$$\mathbf{H}_{\text{pos}} = \begin{bmatrix} \mathbf{I}_{3 \times 3} & \mathbf{0}_{3 \times 12} \end{bmatrix} \in \mathbb{R}^{3 \times 15}$$

## Error Injection and Covariance Reset

Following the Kalman gain computation $\mathbf{K} = \mathbf{P} \mathbf{H}^T (\mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R})^{-1}$, the error state $\delta \mathbf{x} = \mathbf{K} (\mathbf{z} - \hat{\mathbf{z}})$ is injected into the nominal state:
- $\mathbf{p} \leftarrow \mathbf{p} + \delta \mathbf{p}$
- $\mathbf{v} \leftarrow \mathbf{v} + \delta \mathbf{v}$
- $\mathbf{q} \leftarrow \mathbf{q} \otimes \Delta \mathbf{q}(\delta \boldsymbol{\theta})$
- $\mathbf{b}_a \leftarrow \mathbf{b}_a + \delta \mathbf{b}_a$
- $\mathbf{b}_g \leftarrow \mathbf{b}_g + \delta \mathbf{b}_g$

After injection, the error state expectation resets to zero, preventing non-linear divergence.
