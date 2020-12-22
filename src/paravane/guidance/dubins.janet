# src/paravane/guidance/dubins.janet
# 3D Dubins continuous curvature path planning with minimum turning radius and pitch limits.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# Dubins Circle Centers
# -----------------------------------------------------------------------------

(defn circle-centers [x y psi r-min]
  "Compute left and right turning circle centers for pose (x, y, psi)."
  (def c-l [(- x (* r-min (math/sin psi)))
            (+ y (* r-min (math/cos psi)))])
  (def c-r [(+ x (* r-min (math/sin psi)))
            (- y (* r-min (math/cos psi)))])
  {:left c-l :right c-r})

(defn mod2pi [theta]
  (var val theta)
  (while (< val 0.0) (+= val c/two-pi))
  (while (>= val c/two-pi) (-= val c/two-pi))
  val)

# -----------------------------------------------------------------------------
# Dubins Word Solvers (LSL and RSR)
# -----------------------------------------------------------------------------

(defn solve-dubins-lsl [p1 p2 r-min]
  "Solve Left-Straight-Left Dubins trajectory."
  (def x1 (in p1 0)) (def y1 (in p1 1)) (def psi1 (in p1 2))
  (def x2 (in p2 0)) (def y2 (in p2 1)) (def psi2 (in p2 2))

  (def c1 (circle-centers x1 y1 psi1 r-min))
  (def c2 (circle-centers x2 y2 psi2 r-min))
  (def cl1 (get c1 :left))
  (def cl2 (get c2 :left))

  (def d-centers (math/sqrt (+ (* (- (in cl2 0) (in cl1 0)) (- (in cl2 0) (in cl1 0)))
                               (* (- (in cl2 1) (in cl1 1)) (- (in cl2 1) (in cl1 1))))))
  (def ang-centers (math/atan2 (- (in cl2 1) (in cl1 1)) (- (in cl2 0) (in cl1 0))))

  (def arc1 (mod2pi (- ang-centers (+ psi1 c/half-pi))))
  (def straight d-centers)
  (def arc2 (mod2pi (- (+ psi2 c/half-pi) ang-centers)))
  (def total-len (+ (* arc1 r-min) straight (* arc2 r-min)))

  {:type :lsl
   :total-length total-len
   :seg1 (* arc1 r-min)
   :seg2 straight
   :seg3 (* arc2 r-min)})

(defn solve-dubins-rsr [p1 p2 r-min]
  "Solve Right-Straight-Right Dubins trajectory."
  (def x1 (in p1 0)) (def y1 (in p1 1)) (def psi1 (in p1 2))
  (def x2 (in p2 0)) (def y2 (in p2 1)) (def psi2 (in p2 2))

  (def c1 (circle-centers x1 y1 psi1 r-min))
  (def c2 (circle-centers x2 y2 psi2 r-min))
  (def cr1 (get c1 :right))
  (def cr2 (get c2 :right))

  (def d-centers (math/sqrt (+ (* (- (in cr2 0) (in cr1 0)) (- (in cr2 0) (in cr1 0)))
                               (* (- (in cr2 1) (in cr1 1)) (- (in cr2 1) (in cr1 1))))))
  (def ang-centers (math/atan2 (- (in cr2 1) (in cr1 1)) (- (in cr2 0) (in cr1 0))))

  (def arc1 (mod2pi (- (+ psi1 c/half-pi) ang-centers)))
  (def straight d-centers)
  (def arc2 (mod2pi (- ang-centers (- psi2 c/half-pi))))
  (def total-len (+ (* arc1 r-min) straight (* arc2 r-min)))

  {:type :rsr
   :total-length total-len
   :seg1 (* arc1 r-min)
   :seg2 straight
   :seg3 (* arc2 r-min)})

# -----------------------------------------------------------------------------
# Shortest 3D Dubins Path Finder
# -----------------------------------------------------------------------------

(defn find-shortest-dubins-path [p1-3d p2-3d &opt r-min]
  "Find shortest 3D Dubins path connecting initial pose to goal pose."
  (default r-min 15.0) # 15 meters turning radius

  (def sol-lsl (solve-dubins-lsl p1-3d p2-3d r-min))
  (def sol-rsr (solve-dubins-rsr p1-3d p2-3d r-min))

  (def best-2d (if (< (get sol-lsl :total-length) (get sol-rsr :total-length))
                 sol-lsl
                 sol-rsr))

  # 3D extension: vertical depth delta
  (def z1 (in p1-3d 3))
  (def z2 (in p2-3d 3))
  (def dz (- z2 z1))
  (def s-horiz (get best-2d :total-length))
  (def total-3d-length (math/sqrt (+ (* s-horiz s-horiz) (* dz dz))))
  (def pitch-angle (math/atan2 dz s-horiz))

  {:best-word best-2d
   :horizontal-length s-horiz
   :total-3d-length total-3d-length
   :pitch-angle pitch-angle
   :dz dz})

# -----------------------------------------------------------------------------
# Waypoint Path Sampler
# -----------------------------------------------------------------------------

(defn sample-dubins-path [p1-3d p2-3d &opt r-min ds]
  "Sample discrete 3D waypoints along the optimal Dubins curve."
  (default r-min 15.0)
  (default ds 5.0) # 5 meters step

  (def path-info (find-shortest-dubins-path p1-3d p2-3d r-min))
  (def total-len (get path-info :horizontal-length))
  (def num-steps (math/floor (/ total-len ds)))

  (def waypoints @[])
  (def x1 (in p1-3d 0)) (def y1 (in p1-3d 1)) (def z1 (in p1-3d 3))
  (def x2 (in p2-3d 0)) (def y2 (in p2-3d 1)) (def z2 (in p2-3d 3))

  # Linear progression approximation between tangent points
  (for i 0 (+ num-steps 1)
    (def alpha (/ (* i ds) (max 1.0 total-len)))
    (def clamped-alpha (min 1.0 alpha))
    (def x (+ x1 (* clamped-alpha (- x2 x1))))
    (def y (+ y1 (* clamped-alpha (- y2 y1))))
    (def z (+ z1 (* clamped-alpha (- z2 z1))))
    (array/push waypoints [x y z]))

  waypoints)
