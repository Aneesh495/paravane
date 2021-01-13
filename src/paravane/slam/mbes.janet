# src/paravane/slam/mbes.janet
# Multibeam Echo-Sounder (MBES) swath geometry, acoustic beamforming, and sounding point cloud projection.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# MBES Swath Configuration
# -----------------------------------------------------------------------------

(defn make-mbes-config [&opt num-beams swath-angle-deg ping-rate-hz mounting-offset]
  "Construct Multibeam Echo-Sounder (MBES) swath transducer model."
  (default num-beams 64)          # 64 discrete acoustic beams
  (default swath-angle-deg 120.0) # 120 deg across-track swath opening
  (default ping-rate-hz 5.0)      # 5 pings per second
  (default mounting-offset [0.20 0.0 0.10]) # Transducer mount in vehicle body frame

  (def total-angle (* swath-angle-deg c/deg-to-rad))
  (def half-angle (* 0.5 total-angle))
  (def d-beam (/ total-angle (- num-beams 1)))

  # Pre-compute beam pointing angles relative to vehicle vertical axis (positive starboard)
  (def beam-angles (array/new num-beams))
  (for i 0 num-beams
    (array/push beam-angles (+ (- half-angle) (* i d-beam))))

  {:num-beams num-beams
   :swath-angle total-angle
   :ping-rate ping-rate-hz
   :mount mounting-offset
   :beam-angles (tuple ;beam-angles)})

# -----------------------------------------------------------------------------
# Ray Reflection Sounding Projection
# -----------------------------------------------------------------------------

(defn project-beam-sounding-body [mbes beam-idx range-m]
  "Compute 3D coordinates [x_fwd, y_stbd, z_down] of acoustic seabed strike in vehicle body frame."
  (def th (in (get mbes :beam-angles) beam-idx))
  (def mount (get mbes :mount))

  # Beam points across-track (starboard/port) and down
  (def dx 0.0)
  (def dy (* range-m (math/sin th)))
  (def dz (* range-m (math/cos th)))

  [(+ (in mount 0) dx)
   (+ (in mount 1) dy)
   (+ (in mount 2) dz)])

(defn project-swath-to-ned [mbes auv-pos-ned auv-attitude-r beam-ranges]
  "Project full multibeam swath soundings from vehicle pose into absolute local NED point cloud."
  (def points-ned @[])
  (def n (min (get mbes :num-beams) (length beam-ranges)))

  (for i 0 n
    (def r (in beam-ranges i))
    (when (> r 0.5) # Discard invalid or zero-return beams
      (def pt-body (project-beam-sounding-body mbes i r))
      (def pt-rotated (m/mat3-mul-vec auv-attitude-r pt-body))
      (def pt-ned (m/vec3-add auv-pos-ned pt-rotated))
      (array/push points-ned pt-ned)))

  points-ned)
