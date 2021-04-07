# Subsea Hydrodynamics & Propulsion

Subsea vehicle maneuvering and trajectory tracking are governed by 6-degree-of-freedom (6-DOF) coupled non-linear hydrodynamics based on the Fossen (2011) marine craft equations of motion.

## 6-DOF Marine Craft Equations of Motion

In the SNAME (1950) convention, the vehicle body coordinate system is defined with origin at the center of gravity or vehicle design center:
- Surge ($u$, $X$): Longitudinal translation along body x-axis (forward).
- Sway ($v$, $Y$): Transverse translation along body y-axis (starboard).
- Heave ($w$, $Z$): Vertical translation along body z-axis (downward).
- Roll ($p$, $K$): Rotation about body x-axis.
- Pitch ($q$, $M$): Rotation about body y-axis.
- Yaw ($r$, $N$): Rotation about body z-axis.

The non-linear matrix equations of motion are expressed as:

$$\mathbf{M} \dot{\boldsymbol{\nu}} + \mathbf{C}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{D}(\boldsymbol{\nu})\boldsymbol{\nu} + \mathbf{g}(\boldsymbol{\eta}) = \boldsymbol{\tau}$$

where:
- $\boldsymbol{\nu} = [u, v, w, p, q, r]^T \in \mathbb{R}^6$ is the body velocity vector.
- $\boldsymbol{\eta} = [x, y, z, \phi, \theta, \psi]^T \in \mathbb{R}^6$ is the NED pose vector.
- $\boldsymbol{\tau} \in \mathbb{R}^6$ is the control generalized forces and moments vector.

## System Matrices

### 1. Total Inertia Matrix $\mathbf{M} = \mathbf{M}_{RB} + \mathbf{M}_A$
The total mass matrix combines rigid body inertia $\mathbf{M}_{RB}$ with hydrodynamic added mass $\mathbf{M}_A$ (the kinetic energy of the entrained fluid accelerated alongside the moving hull):

$$\mathbf{M}_A = -\text{diag}\left( X_{\dot{u}}, Y_{\dot{v}}, Z_{\dot{w}}, K_{\dot{p}}, M_{\dot{q}}, N_{\dot{r}} \right)$$

For streamlined torpedo-class AUV hulls (e.g. REMUS, HUGIN), lateral and vertical added mass terms $Y_{\dot{v}}$ and $Z_{\dot{w}}$ exceed the rigid body dry mass by 80% to 120%, significantly altering transient dynamics compared to terrestrial robotics.

### 2. Hydrodynamic Damping $\mathbf{D}(\boldsymbol{\nu}) = \mathbf{D}_L + \mathbf{D}_{NL}(\boldsymbol{\nu})$
Damping consists of linear skin friction skin terms and non-linear crossflow drag:

$$\mathbf{D}_L = -\text{diag}\left( X_u, Y_v, Z_w, K_p, M_q, N_r \right)$$

$$\mathbf{D}_{NL}(\boldsymbol{\nu}) = -\text{diag}\left( X_{u|u|}|u|, Y_{v|v|}|v|, Z_{w|w|}|w|, K_{p|p|}|p|, M_{q|q|}|q|, N_{r|r|}|r| \right)$$

### 3. Restoring Forces and Moments $\mathbf{g}(\boldsymbol{\eta})$
Subsea vehicles rely on static metacentric stability, where the center of buoyancy $\mathbf{r}_B = [x_b, y_b, z_b]^T$ is positioned strictly above the center of gravity $\mathbf{r}_G = [x_g, y_g, z_g]^T$ ($z_b < z_g$ in NED).

The restoring gravitational and buoyant forces vector is:

$$\mathbf{g}(\boldsymbol{\eta}) = \begin{bmatrix}
(W - B) \sin\theta \\
-(W - B) \cos\theta \sin\phi \\
-(W - B) \cos\theta \cos\phi \\
-(y_g W - y_b B) \cos\theta \cos\phi + (z_g W - z_b B) \cos\theta \sin\phi \\
(z_g W - z_b B) \sin\theta + (x_g W - x_b B) \cos\theta \cos\phi \\
-(x_g W - x_b B) \cos\theta \sin\phi - (y_g W - y_b B) \sin\theta
\end{bmatrix}$$

where $W = m \cdot g$ is vehicle dry weight, and $B = \rho \cdot \nabla \cdot g$ is buoyant displacement.

## Thruster Allocation Matrix (TAM)

The control generalized force vector $\boldsymbol{\tau} \in \mathbb{R}^6$ is mapped from individual thruster thrusts $\mathbf{u} \in \mathbb{R}^m$ via the Thruster Allocation Matrix $\mathbf{B} \in \mathbb{R}^{6 \times m}$:

$$\boldsymbol{\tau} = \mathbf{B} \mathbf{u}$$

For overactuated survey vehicles equipped with multiple horizontal and vertical tunnel thrusters, Paravane solves the optimal thrust allocation using Moore-Penrose pseudo-inverse with quadratic energy minimization:

$$\mathbf{u}^* = \mathbf{B}^T (\mathbf{B} \mathbf{B}^T)^{-1} \boldsymbol{\tau}_{\text{cmd}}$$

Individual thruster saturation limits are enforced via directional scaling, preserving the desired moment ratio to maintain vehicle heading stability.
