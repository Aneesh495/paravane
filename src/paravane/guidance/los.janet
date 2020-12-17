# src/paravane/guidance/los.janet
# 3D Line-of-Sight (LOS) path-following guidance with adaptive lookahead and ocean drift compensation.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# 3D Waypoint Representation
# -----------------------------------------------------------------------------

(defn make-waypoint [north east depth &opt speed-m-s radius-m]
  (default speed-m-s 1.5) # 1.5 m/s standard survey cruising speed
  (default radius-m 10.0)  # 10m acceptance radius
  {:pos [north east depth]
   :speed speed-m-s
   :radius radius-m})

# -----------------------------------------------------------------------------
# Horizontal LOS Guidance
# -----------------------------------------------------------------------------

(defn compute-cross-track-error-2d [p-curr p-start p-end]
  "Compute along-track distance s and cross-track error e relative to line segment from p_start to p_end."
  (def dx (- (in p-end 1) (in p-start 1))) # East diff
  (def dy (- (in p-end 0) (in p-start 0))) # North diff
  (def path-angle (math/atan2 dx dy)) # Azimuth clockwise from North

  (def rx (- (in p-curr 1) (in p-start 1)))
  (def ry (- (in p-curr 0) (in p-start 0)))

  # Along-track: s = ry * cos(alpha) + rx * sin(alpha)
  (def s (+ (* ry (math/cos path-angle)) (* rx (math/sin path-angle))))
  # Cross-track: e = -ry * sin(alpha) + rx * cos(alpha) (positive e means vehicle is starboard/right of path)
  (def e (+ (* (- ry) (math/sin path-angle)) (* rx (math/cos path-angle))))

  {:s s :e e :path-angle path-angle})

(defn los-desired-heading [p-curr p-start p-end &opt lookahead-dist sideslip-drift]
  "Calculate desired yaw heading using Line-of-Sight guidance law."
  (default lookahead-dist 25.0) # 25 meters standard lookahead
  (default sideslip-drift 0.0)  # Estimated current sideslip angle in radians

  (def geom (compute-cross-track-error-2d p-curr p-start p-end))
  (def path-ang (get geom :path-angle))
  (def e (get geom :e))

  # LOS steering law: psi_d = path_angle - atan(e / Delta) - beta_drift
  (def delta-heading (- (math/atan2 e lookahead-dist)))
  (def psi-des (- (+ path-ang delta-heading) sideslip-drift))

  # Wrap angle to [-pi, pi]
  (var wrapped psi-des)
  (while (> wrapped math/pi) (-= wrapped c/two-pi))
  (while (< wrapped (- math/pi)) (+= wrapped c/two-pi))

  {:desired-heading wrapped
   :cross-track-error e
   :along-track-dist (get geom :s)
   :path-angle path-ang})

# -----------------------------------------------------------------------------
# Vertical Plane LOS Guidance (Depth / Altitude Control)
# -----------------------------------------------------------------------------

(defn los-desired-pitch [z-curr z-target &opt delta-z max-pitch-deg]
  "Calculate desired pitch angle to converge smoothly to target depth."
  (default delta-z 15.0)        # 15 meters vertical lookahead
  (default max-pitch-deg 20.0)  # 20 deg maximum pitch angle

  (def z-err (- z-curr z-target)) # Positive error means AUV is deeper than target
  # Pitch: nose down is negative theta in standard aviation/marine convention, diving increases depth
  # If z_err > 0 (too deep), we want nose up (positive pitch)
  (def raw-pitch (math/atan2 z-err delta-z))
  (def max-pitch (* max-pitch-deg c/deg-to-rad))

  # Clamp to safe pitch envelope
  (max (- max-pitch) (min max-pitch raw-pitch)))

# -----------------------------------------------------------------------------
# Waypoint Acceptance Check
# -----------------------------------------------------------------------------

(defn has-reached-waypoint [p-curr waypoint]
  "Determine whether vehicle is within acceptance sphere of target waypoint."
  (def wp-pos (get waypoint :pos))
  (def d (m/vec3-dist p-curr wp-pos))
  (<= d (get waypoint :radius)))
