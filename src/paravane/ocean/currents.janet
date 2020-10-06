# src/paravane/ocean/currents.janet
# Hydrodynamic ocean current models: Ekman transport, vertical shear, and tidal ellipses.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Coriolis Parameter
# -----------------------------------------------------------------------------

(defn coriolis-f [lat-rad]
  "Compute planetary Coriolis frequency f = 2 * Omega * sin(phi) in rad/s."
  (* 2.0 c/earth-omega (math/sin lat-rad)))

# -----------------------------------------------------------------------------
# Ekman Spiral Current Model
# -----------------------------------------------------------------------------
# Reference:
#   Ekman, V.W. (1905). "On the influence of the Earth's rotation on ocean-currents."
#   Arkiv for Matematik, Astronomi och Fysik, 2(11), 1-52.
# -----------------------------------------------------------------------------

(defn ekman-layer-depth [wind-speed-m-s lat-rad &opt eddy-viscosity]
  "Calculate Ekman layer depth D_E in meters where wind influence decays."
  (default eddy-viscosity 0.05) # m^2/s vertical eddy viscosity Az
  (def f (math/abs (coriolis-f lat-rad)))
  (if (< f 1e-6)
    50.0 # Equatorial fallback depth
    (* math/pi (math/sqrt (/ (* 2.0 eddy-viscosity) f)))))

(defn ekman-spiral-velocity [wind-speed-m-s wind-dir-rad depth-m lat-rad]
  "Compute 2D wind-driven current vector [u_east, v_north] at depth via Ekman spiral."
  (def d-e (ekman-layer-depth wind-speed-m-s lat-rad))
  # Surface drift velocity: typically ~1.5% to 3.0% of wind speed
  (def v0 (* 0.02 wind-speed-m-s))
  (def a (/ depth-m d-e))
  (def decay (math/exp (- a)))

  # In Northern Hemisphere (lat > 0), surface current is 45 deg to right of wind direction
  # In Southern Hemisphere (lat < 0), surface current is 45 deg to left of wind direction
  (def hemisphere-sign (if (>= lat-rad 0.0) 1.0 -1.0))
  (def surface-deflection (* hemisphere-sign (/ math/pi 4.0)))

  # Angle turns clockwise with depth in Northern Hemisphere
  (def current-dir (- wind-dir-rad surface-deflection (* hemisphere-sign a)))
  (def speed (* v0 decay))

  (def u-east (* speed (math/sin current-dir)))
  (def v-north (* speed (math/cos current-dir)))
  [u-east v-north 0.0])

# -----------------------------------------------------------------------------
# Tidal Harmonic Ellipse Current
# -----------------------------------------------------------------------------

(defn tidal-current-m2 [time-s semi-major semi-minor inclination-rad phase-rad]
  "Compute semidiurnal M2 tidal current vector [u, v] in m/s at elapsed time."
  # M2 lunar principal semidiurnal period: 12.4206012 hours = 44714.16 seconds
  (def omega (/ c/two-pi 44714.16))
  (def phase (+ (* omega time-s) phase-rad))

  # Elliptical coordinates
  (def x (* semi-major (math/cos phase)))
  (def y (* semi-minor (math/sin phase)))

  # Rotate by inclination angle clockwise from North
  (def cos-inc (math/cos inclination-rad))
  (def sin-inc (math/sin inclination-rad))
  (def u (+ (* x sin-inc) (* y cos-inc)))
  (def v (- (* x cos-inc) (* y sin-inc)))
  [u v 0.0])

# -----------------------------------------------------------------------------
# Composite Ocean Current Environment
# -----------------------------------------------------------------------------

(defn make-current-model [&opt background-u background-v wind-spd wind-dir tidal-major tidal-minor]
  "Construct an ocean current model combining background, wind-driven, and tidal components."
  (default background-u 0.1)  # 0.1 m/s eastward background drift
  (default background-v -0.05) # 0.05 m/s southward background drift
  (default wind-spd 8.0)       # 8 m/s surface wind
  (default wind-dir 0.785)     # Wind blowing toward Northeast (45 deg)
  (default tidal-major 0.25)   # 0.25 m/s peak M2 tidal current
  (default tidal-minor 0.08)   # 0.08 m/s cross-axis tidal flow
  {:bg-u background-u
   :bg-v background-v
   :wind-speed wind-spd
   :wind-dir wind-dir
   :tidal-major tidal-major
   :tidal-minor tidal-minor})

(defn current-at-depth [model depth-m time-s lat-rad]
  "Query total 3D ocean current vector [u_east, v_north, w_up] at depth and time."
  (def bg [(get model :bg-u) (get model :bg-v) 0.0])
  (def ekman (ekman-spiral-velocity (get model :wind-speed)
                                    (get model :wind-dir)
                                    depth-m
                                    lat-rad))
  (def tide (tidal-current-m2 time-s
                              (get model :tidal-major)
                              (get model :tidal-minor)
                              0.5
                              0.0))

  # Total current vector sum
  [(+ (in bg 0) (in ekman 0) (in tide 0))
   (+ (in bg 1) (in ekman 1) (in tide 1))
   0.0])
