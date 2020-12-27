# src/paravane/guidance/survey.janet
# Boustrophedon lawnmower survey grid planner and cross-tie line generator for subsea mapping.

(import ../core/constants :as c)
(import ../core/math :as m)
(import ./los :as los)

# -----------------------------------------------------------------------------
# Swath Width and Lane Spacing Calculation
# -----------------------------------------------------------------------------

(defn compute-swath-lane-spacing [altitude-m swath-opening-deg &opt overlap-fraction]
  "Calculate optimal survey trackline lane spacing for multibeam sonar mapping."
  (default overlap-fraction 0.25) # 25% swath overlap for complete seafloor coverage

  # Swath width on flat seabed: W_swath = 2 * altitude * tan(theta_opening / 2)
  (def half-angle (* (* 0.5 swath-opening-deg) c/deg-to-rad))
  (def swath-width (* 2.0 altitude-m (math/tan half-angle)))

  # Net lane spacing accounting for required overlap
  (* swath-width (- 1.0 overlap-fraction)))

# -----------------------------------------------------------------------------
# Lawnmower Boustrophedon Grid Generator
# -----------------------------------------------------------------------------

(defn generate-survey-grid [origin-ned length-m width-m lane-spacing-m heading-rad target-depth-m &opt cruising-speed]
  "Generate complete lawnmower survey waypoint mission."
  (default cruising-speed 1.5) # 1.5 m/s standard survey speed

  (def num-lanes (+ 1 (math/floor (/ width-m lane-spacing-m))))
  (def waypoints @[])

  (def cos-head (math/cos heading-rad))
  (def sin-head (math/sin heading-rad))
  # Cross-heading unit vector (perpendicular to trackline, pointing starboard):
  (def cos-cross (math/cos (+ heading-rad c/half-pi)))
  (def sin-cross (math/sin (+ heading-rad c/half-pi)))

  (def ox (in origin-ned 0)) # North origin
  (def oy (in origin-ned 1)) # East origin

  (for lane 0 num-lanes
    (def cross-offset (* lane lane-spacing-m))
    (def lane-ox (+ ox (* cross-offset cos-cross)))
    (def lane-oy (+ oy (* cross-offset sin-cross)))

    # Alternating direction: even lanes go forward, odd lanes go reverse
    (def is-forward (zero? (% lane 2)))
    (def p-start (if is-forward
                   [lane-ox lane-oy target-depth-m]
                   [(+ lane-ox (* length-m cos-head)) (+ lane-oy (* length-m sin-head)) target-depth-m]))
    (def p-end (if is-forward
                 [(+ lane-ox (* length-m cos-head)) (+ lane-oy (* length-m sin-head)) target-depth-m]
                 [lane-ox lane-oy target-depth-m]))

    (array/push waypoints (los/make-waypoint (in p-start 0) (in p-start 1) (in p-start 2) cruising-speed 10.0))
    (array/push waypoints (los/make-waypoint (in p-end 0) (in p-end 1) (in p-end 2) cruising-speed 10.0)))

  waypoints)

# -----------------------------------------------------------------------------
# Cross-Tie Check Line Generator
# -----------------------------------------------------------------------------

(defn generate-cross-tie-lines [origin-ned length-m width-m heading-rad target-depth-m &opt num-ties]
  "Generate orthogonal cross-tie check lines across survey grid to calibrate bathymetric datum."
  (default num-ties 2)

  (def tie-waypoints @[])
  (def cross-heading (+ heading-rad c/half-pi))
  (def cos-cross (math/cos cross-heading))
  (def sin-cross (math/sin cross-heading))
  (def cos-head (math/cos heading-rad))
  (def sin-head (math/sin heading-rad))

  (def ox (in origin-ned 0))
  (def oy (in origin-ned 1))

  (for i 0 num-ties
    (def along-offset (* (/ (+ i 1) (+ num-ties 1)) length-m))
    (def tie-start-x (+ ox (* along-offset cos-head)))
    (def tie-start-y (+ oy (* along-offset sin-head)))
    (def tie-end-x (+ tie-start-x (* width-m cos-cross)))
    (def tie-end-y (+ tie-start-y (* width-m sin-cross)))

    (array/push tie-waypoints (los/make-waypoint tie-start-x tie-start-y target-depth-m 1.5 10.0))
    (array/push tie-waypoints (los/make-waypoint tie-end-x tie-end-y target-depth-m 1.5 10.0)))

  tie-waypoints)
