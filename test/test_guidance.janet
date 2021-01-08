# test/test_guidance.janet
# Comprehensive unit tests for LOS guidance, Dubins curves, survey planning, and obstacle avoidance.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/guidance/los :as los)
(import ../src/paravane/guidance/dubins :as dubins)
(import ../src/paravane/guidance/survey :as survey)
(import ../src/paravane/guidance/terrain_following :as terrain)
(import ../src/paravane/guidance/obstacle_avoidance :as avoid)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# Line-of-Sight (LOS) Path Following Tests
# -----------------------------------------------------------------------------

# Path runs due North from [0, 0] to [1000, 0]
(def p-start [0.0 0.0 50.0])
(def p-end [1000.0 0.0 50.0])

# Vehicle is displaced 10m East (starboard of track) at [100, 10, 50]
(def p-veh [100.0 10.0 50.0])
(def los-res (los/los-desired-heading p-veh p-start p-end 25.0 0.0))

(h/assert-near 10.0 (get los-res :cross-track-error) 1e-6 "Cross-track error is +10m East")
(h/assert-near 100.0 (get los-res :along-track-dist) 1e-6 "Along-track distance is 100m North")
# Vehicle is to the right of path, so desired heading must steer left (negative yaw / West of North)
(h/assert-true (< (get los-res :desired-heading) 0.0) "LOS heading steers left toward path when displaced right")
(h/assert-near (- (math/atan2 10.0 25.0)) (get los-res :desired-heading) 1e-6 "LOS heading matches arctan(e/Delta)")

# Vertical LOS pitch test
# Vehicle at 60m depth, target depth 50m (vehicle is 10m too deep)
(def pitch-cmd (los/los-desired-pitch 60.0 50.0 15.0 20.0))
# Nose up (positive pitch) required to climb
(h/assert-true (> pitch-cmd 0.0) "Vertical LOS commands positive pitch (nose up) to climb to target depth")

# -----------------------------------------------------------------------------
# 3D Dubins Curves Tests
# -----------------------------------------------------------------------------

(def pose-start [0.0 0.0 0.0 50.0])     # (0, 0), heading North, depth 50m
(def pose-goal [100.0 100.0 c/half-pi 70.0]) # (100, 100), heading East, depth 70m

(def dubins-path (dubins/find-shortest-dubins-path pose-start pose-goal 15.0))
(h/assert-true (> (get dubins-path :horizontal-length) 100.0) "Dubins horizontal length > straight line")
(h/assert-true (> (get dubins-path :total-3d-length) (get dubins-path :horizontal-length)) "3D length includes vertical dive")
(h/assert-near 20.0 (get dubins-path :dz) 1e-6 "Vertical delta is 20m depth change")

(def samples (dubins/sample-dubins-path pose-start pose-goal 15.0 10.0))
(h/assert-true (> (length samples) 5) "Dubins path sampler generates multiple discrete trajectory points")

# -----------------------------------------------------------------------------
# Boustrophedon Survey Grid Generator Tests
# -----------------------------------------------------------------------------

# Multibeam sonar 120-degree opening at 20m altitude with 20% overlap
(def lane-sp (survey/compute-swath-lane-spacing 20.0 120.0 0.20))
# W_swath = 2 * 20 * tan(60 deg) = 40 * 1.732 = 69.28m; 80% is ~55.4m
(h/assert-near 55.42 lane-sp 0.5 "Multibeam lane spacing at 20m altitude with 20% overlap is ~55.4m")

# Generate 500m x 200m survey grid
(def grid-wps (survey/generate-survey-grid [0.0 0.0 100.0] 500.0 200.0 50.0 0.0 100.0))
# 200m width / 50m spacing = 4 intervals -> 5 lanes -> 10 waypoints (start and end per lane)
(h/assert-equal 10 (length grid-wps) "5-lane survey grid produces exactly 10 trackline waypoints")

# Cross-tie line generator test
(def ties (survey/generate-cross-tie-lines [0.0 0.0 100.0] 500.0 200.0 0.0 100.0 2))
(h/assert-equal 4 (length ties) "2 cross-tie check lines generate 4 endpoints")

# -----------------------------------------------------------------------------
# Terrain Following Altitude Hold Controller Tests
# -----------------------------------------------------------------------------

(def alt-ctrl (terrain/make-terrain-controller 15.0 0.05 0.08 0.002 20.0 4.0))

# Test Case 1: Operating at steady 15m altitude on flat seabed
(def res-flat (terrain/update-altitude-hold alt-ctrl 15.0 0.0 0.0 0.1))
(h/assert-near 0.0 (get res-flat :pitch-cmd) 1e-6 "Zero pitch command when altitude is on target on flat seabed")
(h/assert-false (get res-flat :emergency-flare) "No emergency flare under nominal altitude")

# Test Case 2: Approaching rising seabed slope (slope = +0.10 rad)
(def res-slope (terrain/update-altitude-hold alt-ctrl 15.0 0.0 0.10 0.1))
(h/assert-near 0.10 (get res-slope :pitch-cmd) 1e-3 "Feedforward pitch matches upward seabed slope")

# Test Case 3: Emergency bottom avoidance (altitude drops to 3.5m < 4.0m critical threshold)
(def res-emergency (terrain/update-altitude-hold alt-ctrl 3.5 -0.5 0.0 0.1))
(h/assert-true (get res-emergency :emergency-flare) "Emergency bottom flare triggered when altitude < 4.0m")
(h/assert-near (* 20.0 c/deg-to-rad) (get res-emergency :pitch-cmd) 1e-6 "Emergency flare commands maximum pitch-up")

# -----------------------------------------------------------------------------
# Forward-Looking Sonar (FLS) Obstacle Avoidance Tests
# -----------------------------------------------------------------------------

(def fls (avoid/make-fls-config 120.0 90.0 30.0 25.0))

# Clear ocean test (no contacts within 25m)
(def clear-res (avoid/check-obstacle-conflicts fls [[80.0 20.0 0.0] [95.0 -10.0 5.0]]))
(h/assert-false (get clear-res :threat) "No collision threat when all obstacles are > 25m away")

# Threat detected: subsea pinnacle 18m forward, 5m to starboard
(def threat-contacts [[18.0 5.0 0.0]])
(def threat-res (avoid/check-obstacle-conflicts fls threat-contacts))
(h/assert-true (get threat-res :threat) "Collision threat detected for contact at 18m")

# Evasive maneuver test: obstacle is to starboard, vehicle must steer port (turn left)
(def evade-res (avoid/compute-evasive-heading 0.0 fls threat-res 30.0))
(h/assert-true (get evade-res :evade) "Evasive maneuver activated")
(h/assert-true (< (get evade-res :delta-heading) 0.0) "Evasive steering turns port away from starboard obstacle")

(h/summarize-tests "Subsea Guidance, Dubins 3D Curves, Survey Grids, and Obstacle Avoidance")
