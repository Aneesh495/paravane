# src/paravane/navigation/strapdown.janet
# High-rate strapdown inertial navigation system (INS) mechanization in local NED frame.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# INS State Structure
# -----------------------------------------------------------------------------

(defn make-ins-state [&opt pos-ned vel-ned quat accel-bias gyro-bias]
  "Construct an initial strapdown inertial navigation state."
  (default pos-ned [0.0 0.0 0.0])
  (default vel-ned [0.0 0.0 0.0])
  (default quat m/quat-identity)
  (default accel-bias [0.0 0.0 0.0])
  (default gyro-bias [0.0 0.0 0.0])
  {:pos pos-ned
   :vel vel-ned
   :quat (m/quat-normalize quat)
   :accel-bias accel-bias
   :gyro-bias gyro-bias})

# -----------------------------------------------------------------------------
# Strapdown Mechanization Step (Local NED Tangent Plane)
# -----------------------------------------------------------------------------

(defn ins-mechanize-step [state raw-accel raw-omega dt &opt lat-rad]
  "Propagate strapdown navigation state forward by dt seconds given IMU accelerometer and gyroscope readings."
  (default lat-rad (* 45.0 c/deg-to-rad))

  (def pos (get state :pos))
  (def vel (get state :vel))
  (def q (get state :quat))
  (def ba (get state :accel-bias))
  (def bg (get state :gyro-bias))

  # 1. Correct sensor biases
  (def f-body (m/vec3-sub raw-accel ba))
  (def omega-body (m/vec3-sub raw-omega bg))

  # 2. Attitude update: integrate quaternion
  (def next-q (m/quat-integrate q omega-body dt))

  # 3. Specific force rotation to local NED frame
  # Use mid-point rotation matrix for 2nd order accuracy
  (def q-mid (m/quat-slerp q next-q 0.5))
  (def r-nb (m/quat-to-mat3 q-mid))
  (def f-ned (m/mat3-mul-vec r-nb f-body))

  # 4. Gravity vector in NED: [0, 0, g(depth)]
  (def depth (in pos 2))
  (def g-local (c/gravity-at-depth lat-rad depth))
  (def g-ned [0.0 0.0 g-local])

  # 5. Earth Coriolis acceleration: -2 * Omega_ie x v_ned
  # In local NED: Omega_ie = [Omega * cos(lat), 0, -Omega * sin(lat)]
  (def omega-ie [(* c/earth-omega (math/cos lat-rad))
                 0.0
                 (* -1.0 c/earth-omega (math/sin lat-rad))])
  (def coriolis (m/vec3-scale (m/vec3-cross omega-ie vel) -2.0))

  # 6. Total linear acceleration in NED frame
  (def a-ned (m/vec3-add f-ned (m/vec3-add g-ned coriolis)))

  # 7. Velocity integration (trapezoidal rule)
  (def next-vel (m/vec3-add vel (m/vec3-scale a-ned dt)))

  # 8. Position integration
  (def v-mid (m/vec3-scale (m/vec3-add vel next-vel) 0.5))
  (def next-pos (m/vec3-add pos (m/vec3-scale v-mid dt)))

  {:pos next-pos
   :vel next-vel
   :quat next-q
   :accel-bias ba
   :gyro-bias bg})
