# src/paravane/navigation/dvl.janet
# 4-beam Janus Doppler Velocity Log (DVL) beam-to-body coordinate transformations and sound speed correction.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# Janus 4-Beam Geometry (Standard Plus Configuration: Fwd, Stbd, Aft, Port)
# -----------------------------------------------------------------------------

(defn make-janus-dvl [&opt beam-angle-deg sound-speed-cal]
  "Construct Janus 4-beam DVL geometry."
  (default beam-angle-deg 30.0) # 30 deg beam inclination from vertical
  (default sound-speed-cal 1500.0) # Factory calibration sound speed

  (def alpha (* beam-angle-deg c/deg-to-rad))
  (def sin-a (math/sin alpha))
  (def cos-a (math/cos alpha))

  # Beam unit direction vectors in instrument frame (pointing downward into water):
  # Beam 1: Forward  (azimuth = 0 deg)
  (def b1 [sin-a 0.0 cos-a])
  # Beam 2: Starboard (azimuth = 90 deg)
  (def b2 [0.0 sin-a cos-a])
  # Beam 3: Aft      (azimuth = 180 deg)
  (def b3 [(- sin-a) 0.0 cos-a])
  # Beam 4: Port     (azimuth = 270 deg)
  (def b4 [0.0 (- sin-a) cos-a])

  {:alpha alpha
   :sin-alpha sin-a
   :cos-alpha cos-a
   :c-cal sound-speed-cal
   :b1 b1
   :b2 b2
   :b3 b3
   :b4 b4})

# -----------------------------------------------------------------------------
# Forward Kinematics: Body Velocity to Beam Velocities
# -----------------------------------------------------------------------------

(defn body-to-beam-velocities [dvl v-body]
  "Project 3D vehicle body velocity [u, v, w] onto 4 acoustic Janus beams."
  [(m/vec3-dot v-body (get dvl :b1))
   (m/vec3-dot v-body (get dvl :b2))
   (m/vec3-dot v-body (get dvl :b3))
   (m/vec3-dot v-body (get dvl :b4))])

# -----------------------------------------------------------------------------
# Inverse Kinematics: 4-Beam Velocities to 3D Body Velocity
# -----------------------------------------------------------------------------

(defn beam-to-body-velocities [dvl beam-vels]
  "Solve for vehicle body velocity [u_surge, v_sway, w_heave] from 4 Janus beam readings."
  (def v1 (in beam-vels 0))
  (def v2 (in beam-vels 1))
  (def v3 (in beam-vels 2))
  (def v4 (in beam-vels 3))

  (def sin-a (get dvl :sin-alpha))
  (def cos-a (get dvl :cos-alpha))

  # In symmetric Janus configuration:
  # u (forward)   = (v1 - v3) / (2 * sin(alpha))
  # v (starboard) = (v2 - v4) / (2 * sin(alpha))
  # w (downward)  = (v1 + v2 + v3 + v4) / (4 * cos(alpha))
  (def u (/ (- v1 v3) (* 2.0 sin-a)))
  (def v (/ (- v2 v4) (* 2.0 sin-a)))
  (def w (/ (+ v1 v2 v3 v4) (* 4.0 cos-a)))

  # Quality metric: error velocity (discrepancy between vertical estimates from orthogonal pairs)
  (def w13 (/ (+ v1 v3) (* 2.0 cos-a)))
  (def w24 (/ (+ v2 v4) (* 2.0 cos-a)))
  (def err-vel (- w13 w24))

  {:vel [u v w]
   :error-velocity err-vel
   :valid true})

# -----------------------------------------------------------------------------
# 3-Beam Fallback Solution (When 1 Beam Drops Lock)
# -----------------------------------------------------------------------------

(defn beam-to-body-3beam-fallback [dvl beam-vels missing-beam-idx]
  "Reconstruct 3D body velocity using 3 valid Janus beams when 1 beam loses lock."
  (def sin-a (get dvl :sin-alpha))
  (def cos-a (get dvl :cos-alpha))

  (def v1 (in beam-vels 0))
  (def v2 (in beam-vels 1))
  (def v3 (in beam-vels 2))
  (def v4 (in beam-vels 3))

  (case missing-beam-idx
    # Beam 1 (Fwd) missing: use v3 for forward, (v2+v4) for vertical
    0 (let [w (/ (+ v2 v4) (* 2.0 cos-a))
            u (/ (- (* w cos-a) v3) sin-a)
            v (/ (- v2 v4) (* 2.0 sin-a))]
        {:vel [u v w] :fallback true :missing 0})
    # Beam 2 (Stbd) missing: use (v1+v3) for vertical, v4 for sway
    1 (let [w (/ (+ v1 v3) (* 2.0 cos-a))
            u (/ (- v1 v3) (* 2.0 sin-a))
            v (/ (- (* w cos-a) v4) sin-a)]
        {:vel [u v w] :fallback true :missing 1})
    # Beam 3 (Aft) missing: use v1 for forward, (v2+v4) for vertical
    2 (let [w (/ (+ v2 v4) (* 2.0 cos-a))
            u (/ (- v1 (* w cos-a)) sin-a)
            v (/ (- v2 v4) (* 2.0 sin-a))]
        {:vel [u v w] :fallback true :missing 2})
    # Beam 4 (Port) missing: use (v1+v3) for vertical, v2 for sway
    3 (let [w (/ (+ v1 v3) (* 2.0 cos-a))
            u (/ (- v1 v3) (* 2.0 sin-a))
            v (/ (- v2 (* w cos-a)) sin-a)]
        {:vel [u v w] :fallback true :missing 3})
    # Default to 4-beam solver
    (beam-to-body-velocities dvl beam-vels)))

# -----------------------------------------------------------------------------
# Sound Speed Scaling Correction
# -----------------------------------------------------------------------------

(defn correct-dvl-sound-speed [measured-vel c-actual c-calibrated]
  "Apply Doppler frequency scaling correction for in-situ seawater sound speed."
  (def scale-factor (/ c-actual c-calibrated))
  (m/vec3-scale measured-vel scale-factor))
