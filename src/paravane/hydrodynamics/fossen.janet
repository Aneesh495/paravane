# src/paravane/hydrodynamics/fossen.janet
# 6-DOF nonlinear underwater vehicle dynamics based on Thor I. Fossen marine craft formulation.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# 6-DOF Marine Craft Kinematics
# -----------------------------------------------------------------------------
# Body velocities: nu = [u (surge), v (sway), w (heave), p (roll), q (pitch), r (yaw)]
# NED positions and Euler angles: eta = [x, y, z, phi, theta, psi]
# -----------------------------------------------------------------------------

(defn euler-rotation-matrix-j1 [phi theta psi]
  "Compute 3x3 linear velocity transformation matrix J1(eta2) = R_nb."
  (m/mat3-from-euler phi theta psi))

(defn euler-angular-matrix-j2 [phi theta]
  "Compute 3x3 angular velocity transformation matrix J2(eta2) mapping [p, q, r] to [phi_dot, theta_dot, psi_dot]."
  (def cphi (math/cos phi))
  (def sphi (math/sin phi))
  (def cth (math/cos theta))
  (def tth (math/tan theta))

  # Avoid singularity near pitch = +/- 90 degrees (gimbal lock)
  (def safe-cth (if (< (math/abs cth) 1e-4) (if (>= cth 0.0) 1e-4 -1e-4) cth))

  [1.0 (* sphi tth) (* cphi tth)
   0.0 cphi         (- sphi)
   0.0 (/ sphi safe-cth) (/ cphi safe-cth)])

(defn nu-to-eta-dot [nu eta]
  "Compute time derivative of NED position and Euler angles: eta_dot = J(eta) * nu."
  (def u (in nu 0)) (def v (in nu 1)) (def w (in nu 2))
  (def p (in nu 3)) (def q (in nu 4)) (def r (in nu 5))

  (def phi (in eta 3))
  (def theta (in eta 4))
  (def psi (in eta 5))

  (def j1 (euler-rotation-matrix-j1 phi theta psi))
  (def v-linear (m/mat3-mul-vec j1 [u v w]))

  (def j2 (euler-angular-matrix-j2 phi theta))
  (def v-angular (m/mat3-mul-vec j2 [p q r]))

  [(in v-linear 0) (in v-linear 1) (in v-linear 2)
   (in v-angular 0) (in v-angular 1) (in v-angular 2)])

# -----------------------------------------------------------------------------
# Restoring Forces and Moments (Gravity and Buoyancy)
# -----------------------------------------------------------------------------

(defn restoring-forces [weight-w buoyancy-b rg rb phi theta]
  "Compute 6-DOF hydrostatic restoring force and moment vector g(eta)."
  (def cphi (math/cos phi))
  (def sphi (math/sin phi))
  (def cth (math/cos theta))
  (def sth (math/sin theta))

  (def xg (in rg 0)) (def yg (in rg 1)) (def zg (in rg 2))
  (def xb (in rb 0)) (def yb (in rb 1)) (def zb (in rb 2))

  (def w-minus-b (- weight-w buoyancy-b))

  # Surge restoring force
  (def g1 (* w-minus-b sth))

  # Sway restoring force
  (def g2 (* -1.0 w-minus-b cth sphi))

  # Heave restoring force
  (def g3 (* -1.0 w-minus-b cth cphi))

  # Roll restoring moment (metacentric righting moment)
  (def g4 (+ (* -1.0 (- (* yg weight-w) (* yb buoyancy-b)) cth cphi)
             (* (- (* zg weight-w) (* zb buoyancy-b)) cth sphi)))

  # Pitch restoring moment
  (def g5 (+ (* (- (* zg weight-w) (* zb buoyancy-b)) sth)
             (* (- (* xg weight-w) (* xb buoyancy-b)) cth cphi)))

  # Yaw restoring moment
  (def g6 (+ (* -1.0 (- (* xg weight-w) (* xb buoyancy-b)) cth sphi)
             (* -1.0 (- (* yg weight-w) (* yb buoyancy-b)) sth)))

  [g1 g2 g3 g4 g5 g6])

# -----------------------------------------------------------------------------
# Hydrodynamic Damping (Linear Skin Friction + Quadratic Form Drag)
# -----------------------------------------------------------------------------

(defn hydrodynamic-damping [d-linear d-quad nu]
  "Compute 6-DOF hydrodynamic damping forces and moments: D(nu) * nu."
  (def res (array/new 6))
  (for i 0 6
    (def vel (in nu i))
    (def f-lin (* (in d-linear i) vel))
    (def f-quad (* (in d-quad i) (math/abs vel) vel))
    (array/push res (+ f-lin f-quad)))
  res)

# -----------------------------------------------------------------------------
# Rigid-Body and Added Mass Acceleration Solver
# -----------------------------------------------------------------------------

(defn solve-nu-dot [m-total-diag c-coriolis-nu damping-forces restoring-g tau-control]
  "Solve for body acceleration nu_dot = M^-1 * (tau_ctrl - C*nu - D*nu - g)."
  (def nu-dot (array/new 6))
  (for i 0 6
    (def net-force (- (in tau-control i)
                      (in c-coriolis-nu i)
                      (in damping-forces i)
                      (in restoring-g i)))
    (def m-i (in m-total-diag i))
    (array/push nu-dot (/ net-force m-i)))
  nu-dot)
