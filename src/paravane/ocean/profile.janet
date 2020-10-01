# src/paravane/ocean/profile.janet
# Sound Velocity Profile (SVP) modeling, cubic spline interpolation, and SOFAR channel detection.

(import ./sound_speed :as ss)

# -----------------------------------------------------------------------------
# SVP Representation
# -----------------------------------------------------------------------------

(defn make-svp [depths speeds &opt temps salinities]
  "Construct an ocean Sound Velocity Profile from depth and sound speed arrays."
  {:depths (tuple ;depths)
   :speeds (tuple ;speeds)
   :temps (if temps (tuple ;temps) nil)
   :salinities (if salinities (tuple ;salinities) nil)})

(defn svp-layer-count [profile]
  (length (get profile :depths)))

(defn svp-min-depth [profile]
  (in (get profile :depths) 0))

(defn svp-max-depth [profile]
  (in (get profile :depths) (- (svp-layer-count profile) 1)))

# -----------------------------------------------------------------------------
# Natural Cubic Spline Interpolation for Continuous Sound Speed Profiles
# -----------------------------------------------------------------------------

(defn build-natural-cubic-spline [xs ys]
  "Build natural cubic spline coefficients a, b, c, d for points (xs[i], ys[i])."
  (def n (- (length xs) 1))
  (def h (array/new n))
  (for i 0 n
    (array/push h (- (in xs (+ i 1)) (in xs i))))

  (def alpha (array/new n))
  (array/push alpha 0.0) # alpha[0] unused
  (for i 1 n
    (def term1 (/ (- (in ys (+ i 1)) (in ys i)) (in h i)))
    (def term2 (/ (- (in ys i) (in ys (- i 1))) (in h (- i 1))))
    (array/push alpha (* 3.0 (- term1 term2))))

  (def l (array/new (+ n 1)))
  (def mu (array/new (+ n 1)))
  (def z (array/new (+ n 1)))

  (array/push l 1.0)
  (array/push mu 0.0)
  (array/push z 0.0)

  (for i 1 n
    (def li (- (* 2.0 (- (in xs (+ i 1)) (in xs (- i 1)))) (* (in h (- i 1)) (in mu (- i 1)))))
    (array/push l li)
    (array/push mu (/ (in h i) li))
    (array/push z (/ (- (in alpha i) (* (in h (- i 1)) (in z (- i 1)))) li)))

  (array/push l 1.0)
  (array/push z 0.0)

  (def c-coeffs (array/new (+ n 1)))
  (for i 0 (+ n 1) (array/push c-coeffs 0.0))

  (def b-coeffs (array/new n))
  (def d-coeffs (array/new n))
  (for i 0 n
    (array/push b-coeffs 0.0)
    (array/push d-coeffs 0.0))

  (for j-rev 1 (+ n 1)
    (def j (- n j-rev))
    (put c-coeffs j (- (in z j) (* (in mu j) (in c-coeffs (+ j 1)))))
    (def hj (in h j))
    (def dy (- (in ys (+ j 1)) (in ys j)))
    (def bj (- (/ dy hj) (* (/ hj 3.0) (+ (* 2.0 (in c-coeffs j)) (in c-coeffs (+ j 1))))))
    (def dj (/ (- (in c-coeffs (+ j 1)) (in c-coeffs j)) (* 3.0 hj)))
    (put b-coeffs j bj)
    (put d-coeffs j dj))

  {:xs xs
   :ys ys
   :b b-coeffs
   :c c-coeffs
   :d d-coeffs
   :n n})

(defn eval-cubic-spline [spline x]
  "Evaluate cubic spline and its first derivative at coordinate x."
  (def xs (get spline :xs))
  (def ys (get spline :ys))
  (def b (get spline :b))
  (def c (get spline :c))
  (def d (get spline :d))
  (def n (get spline :n))

  # Clamp extrapolation to boundary points
  (if (<= x (in xs 0))
    {:val (in ys 0) :deriv (in b 0)}
    (if (>= x (in xs n))
      {:val (in ys n) :deriv (in b (- n 1))}
      (do
        # Binary search for interval
        (var low 0)
        (var high n)
        (while (> (- high low) 1)
          (def mid (math/floor (/ (+ low high) 2)))
          (if (<= (in xs mid) x)
            (set low mid)
            (set high mid)))
        (def dx (- x (in xs low)))
        (def val (+ (in ys low)
                    (* (in b low) dx)
                    (* (in c low) dx dx)
                    (* (in d low) dx dx dx)))
        (def deriv (+ (in b low)
                      (* 2.0 (in c low) dx)
                      (* 3.0 (in d low) dx dx)))
        {:val val :deriv deriv}))))

# -----------------------------------------------------------------------------
# Linear Profile Interpolator
# -----------------------------------------------------------------------------

(defn svp-eval-linear [profile depth]
  "Evaluate sound speed at target depth with piecewise constant gradient."
  (def depths (get profile :depths))
  (def speeds (get profile :speeds))
  (def n (length depths))

  (if (<= depth (in depths 0))
    {:speed (in speeds 0) :gradient 0.0}
    (if (>= depth (in depths (- n 1)))
      {:speed (in speeds (- n 1)) :gradient 0.0}
      (do
        (var idx 0)
        (for i 0 (- n 1)
          (when (and (>= depth (in depths i)) (<= depth (in depths (+ i 1))))
            (set idx i)
            (break)))
        (def z0 (in depths idx))
        (def z1 (in depths (+ idx 1)))
        (def c0 (in speeds idx))
        (def c1 (in speeds (+ idx 1)))
        (def dz (- z1 z0))
        (def g (if (> dz 1e-6) (/ (- c1 c0) dz) 0.0))
        (def c-interp (+ c0 (* g (- depth z0))))
        {:speed c-interp :gradient g}))))

# -----------------------------------------------------------------------------
# Oceanographic Feature Analysis
# -----------------------------------------------------------------------------

(defn find-sofar-axis [profile]
  "Locate the deep sound channel (SOFAR axis) depth where sound speed reaches its minimum."
  (def depths (get profile :depths))
  (def speeds (get profile :speeds))
  (var min-c 1e9)
  (var min-z 0.0)

  (for i 0 (length depths)
    (def c-val (in speeds i))
    (when (< c-val min-c)
      (set min-c c-val)
      (set min-z (in depths i))))
  {:depth min-z :min-speed min-c})

(defn find-surface-mixed-layer [profile &opt delta-c-threshold]
  "Find the depth of the isothermal surface mixed layer (depth where c drops by delta-c-threshold from surface)."
  (default delta-c-threshold 0.5) # 0.5 m/s drop
  (def depths (get profile :depths))
  (def speeds (get profile :speeds))
  (def c-surf (in speeds 0))

  (var mld (in depths (- (length depths) 1)))
  (for i 1 (length depths)
    (def diff (- c-surf (in speeds i)))
    (when (>= diff delta-c-threshold)
      (set mld (in depths i))
      (break)))
  mld)

# -----------------------------------------------------------------------------
# Canonical Munk Ocean Acoustic Profile
# -----------------------------------------------------------------------------
# Reference:
#   Munk, W.H. (1974). "Sound channel in an exponentially stratified ocean."
#   JASA, 55(2), 220-226.
#
# c(z) = c_axis * [1 + epsilon * (eta + exp(-eta) - 1)]
# where eta = 2 * (z - z_axis) / B
# -----------------------------------------------------------------------------

(defn munk-canonical-profile [&opt z-axis c-axis epsilon b-scale max-depth num-points]
  "Generate Walter Munk's canonical sound channel profile."
  (default z-axis 1000.0) # Depth of SOFAR axis in meters (typically 1000m in mid-latitudes)
  (default c-axis 1490.0) # Sound speed at channel axis (m/s)
  (default epsilon 0.00737) # Perturbation parameter
  (default b-scale 1300.0) # Scale depth (meters)
  (default max-depth 4000.0)
  (default num-points 41)

  (def depths (array/new num-points))
  (def speeds (array/new num-points))
  (def dz (/ max-depth (- num-points 1)))

  (for i 0 num-points
    (def z (* i dz))
    (def eta (/ (* 2.0 (- z z-axis)) b-scale))
    (def c-val (* c-axis (+ 1.0 (* epsilon (+ eta (math/exp (- eta)) -1.0)))))
    (array/push depths z)
    (array/push speeds c-val))

  (make-svp depths speeds))
