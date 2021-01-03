# src/paravane/guidance/obstacle_avoidance.janet
# Forward-Looking Sonar (FLS) acoustic sector scanning and reactive obstacle avoidance.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# FLS Sensor Configuration
# -----------------------------------------------------------------------------

(defn make-fls-config [&opt max-range-m fov-azimuth-deg fov-elevation-deg safety-radius-m]
  "Construct Forward-Looking Sonar (FLS) scanning envelope."
  (default max-range-m 120.0)      # 120 meters detection range
  (default fov-azimuth-deg 90.0)   # +/- 45 deg horizontal field of view
  (default fov-elevation-deg 30.0) # +/- 15 deg vertical field of view
  (default safety-radius-m 25.0)   # 25 meters critical avoidance trigger distance
  {:max-range max-range-m
   :fov-azimuth (* fov-azimuth-deg c/deg-to-rad)
   :fov-elevation (* fov-elevation-deg c/deg-to-rad)
   :safety-radius safety-radius-m})

# -----------------------------------------------------------------------------
# Obstacle Conflict Detection
# -----------------------------------------------------------------------------

(defn check-obstacle-conflicts [fls-cfg contacts-rel-body]
  "Assess forward sonar contacts in vehicle body frame [forward, starboard, down] for collision risk."
  (def safety-r (get fls-cfg :safety-radius))
  (var closest-dist 1e9)
  (var critical-contact nil)

  (each pt contacts-rel-body
    (def fwd (in pt 0))
    (def stbd (in pt 1))
    (def d (m/vec3-norm pt))
    # Check if contact is forward of vehicle and within safety envelope
    (when (and (> fwd 0.0) (< d safety-r))
      (when (< d closest-dist)
        (set closest-dist d)
        (set critical-contact pt))))

  (if critical-contact
    {:threat true
     :closest-dist closest-dist
     :contact critical-contact}
    {:threat false
     :closest-dist 1e9
     :contact nil}))

# -----------------------------------------------------------------------------
# Evasive Heading Resolution
# -----------------------------------------------------------------------------

(defn compute-evasive-heading [current-heading fls-cfg threat-info &opt evasion-angle-deg]
  "Calculate emergency deflection heading away from detected subsea obstacle."
  (default evasion-angle-deg 35.0) # 35 degree evasive maneuver

  (if (not (get threat-info :threat))
    {:evade false :heading current-heading}
    (let [contact (get threat-info :contact)
          stbd-offset (in contact 1)
          evade-rad (* evasion-angle-deg c/deg-to-rad)
          # If obstacle is to starboard (stbd > 0), steer port (negative yaw delta)
          # If obstacle is to port (stbd <= 0), steer starboard (positive yaw delta)
          delta-psi (if (> stbd-offset 0.0) (- evade-rad) evade-rad)
          evade-heading (+ current-heading delta-psi)]
      {:evade true
       :heading evade-heading
       :delta-heading delta-psi})))
