# src/paravane/acoustics/ray_tracer.janet
# Hamiltonian acoustic ray tracer using 4th-order Runge-Kutta integration in stratified media.

(import ../core/constants :as c)
(import ../ocean/profile :as prof)

# -----------------------------------------------------------------------------
# Ray Step Representation
# -----------------------------------------------------------------------------

(defn make-ray-step [x z theta tau s-length]
  {:x x
   :z z
   :theta theta
   :tau tau
   :length s-length})

# -----------------------------------------------------------------------------
# Hamiltonian Ray Equations of Motion in Stratified Media
# -----------------------------------------------------------------------------
# Equations of motion parameterized by arc length s:
#   dx/ds   = cos(theta)
#   dz/ds   = sin(theta)
#   dth/ds  = - (1 / c) * (dc/dz) * cos(theta)
#   dtau/ds = 1 / c
# -----------------------------------------------------------------------------

(defn ray-rk4-step [profile x z theta tau ds bottom-z]
  "Perform a 4th-order Runge-Kutta numerical step along the acoustic ray arc length."
  # Stage 1
  (def eval1 (prof/svp-eval-linear profile z))
  (def c1 (get eval1 :speed))
  (def g1 (get eval1 :gradient))
  (def k1-x (math/cos theta))
  (def k1-z (math/sin theta))
  (def k1-th (* (/ -1.0 c1) g1 k1-x))
  (def k1-tau (/ 1.0 c1))

  # Stage 2
  (def z2 (+ z (* 0.5 ds k1-z)))
  (def th2 (+ theta (* 0.5 ds k1-th)))
  (def eval2 (prof/svp-eval-linear profile z2))
  (def c2 (get eval2 :speed))
  (def g2 (get eval2 :gradient))
  (def k2-x (math/cos th2))
  (def k2-z (math/sin th2))
  (def k2-th (* (/ -1.0 c2) g2 k2-x))
  (def k2-tau (/ 1.0 c2))

  # Stage 3
  (def z3 (+ z (* 0.5 ds k2-z)))
  (def th3 (+ theta (* 0.5 ds k2-th)))
  (def eval3 (prof/svp-eval-linear profile z3))
  (def c3 (get eval3 :speed))
  (def g3 (get eval3 :gradient))
  (def k3-x (math/cos th3))
  (def k3-z (math/sin th3))
  (def k3-th (* (/ -1.0 c3) g3 k3-x))
  (def k3-tau (/ 1.0 c3))

  # Stage 4
  (def z4 (+ z (* ds k3-z)))
  (def th4 (+ theta (* ds k3-th)))
  (def eval4 (prof/svp-eval-linear profile z4))
  (def c4 (get eval4 :speed))
  (def g4 (get eval4 :gradient))
  (def k4-x (math/cos th4))
  (def k4-z (math/sin th4))
  (def k4-th (* (/ -1.0 c4) g4 k4-x))
  (def k4-tau (/ 1.0 c4))

  # RK4 weighted combination
  (def dx (* (/ ds 6.0) (+ k1-x (* 2.0 k2-x) (* 2.0 k3-x) k4-x)))
  (def dz (* (/ ds 6.0) (+ k1-z (* 2.0 k2-z) (* 2.0 k3-z) k4-z)))
  (def dth (* (/ ds 6.0) (+ k1-th (* 2.0 k2-th) (* 2.0 k3-th) k4-th)))
  (def dtau (* (/ ds 6.0) (+ k1-tau (* 2.0 k2-tau) (* 2.0 k3-tau) k4-tau)))

  (def nx (+ x dx))
  (var nz (+ z dz))
  (var nth (+ theta dth))
  (def ntau (+ tau dtau))

  # Surface and seabed boundary reflection checks
  (var bounce nil)
  (when (<= nz 0.0)
    (set nz (math/abs nz))
    (set nth (math/abs nth)) # Specular reflection downward
    (set bounce :surface))

  (when (>= nz bottom-z)
    (set nz (- (* 2.0 bottom-z) nz))
    (set nth (- (math/abs nth))) # Specular reflection upward
    (set bounce :bottom))

  {:x nx
   :z nz
   :theta nth
   :tau ntau
   :bounce bounce})

# -----------------------------------------------------------------------------
# Full Acoustic Ray Propagation Tracer
# -----------------------------------------------------------------------------

(defn trace-acoustic-ray [profile z-src theta-src max-range &opt step-size max-time bottom-depth]
  "Trace acoustic ray trajectory through water column from source (0, z_src) with launch angle theta_src."
  (default step-size 10.0) # 10 meters arc length step
  (default max-time 30.0)
  (default bottom-depth (prof/svp-max-depth profile))

  (def c-src (get (prof/svp-eval-linear profile z-src) :speed))
  (def xi (/ (math/cos theta-src) c-src)) # Snell ray parameter

  (def path @[])
  (var cur-x 0.0)
  (var cur-z z-src)
  (var cur-theta theta-src)
  (var cur-tau 0.0)
  (var cur-s 0.0)
  (var surface-bounces 0)
  (var bottom-bounces 0)

  (array/push path (make-ray-step cur-x cur-z cur-theta cur-tau cur-s))

  (while (and (< cur-x max-range) (< cur-tau max-time))
    (def res (ray-rk4-step profile cur-x cur-z cur-theta cur-tau step-size bottom-depth))
    (set cur-x (get res :x))
    (set cur-z (get res :z))
    (set cur-theta (get res :theta))
    (set cur-tau (get res :tau))
    (+= cur-s step-size)

    (when (= (get res :bounce) :surface) (++ surface-bounces))
    (when (= (get res :bounce) :bottom) (++ bottom-bounces))

    (array/push path (make-ray-step cur-x cur-z cur-theta cur-tau cur-s)))

  {:path path
   :final-x cur-x
   :final-z cur-z
   :travel-time cur-tau
   :surface-bounces surface-bounces
   :bottom-bounces bottom-bounces
   :xi xi})

# -----------------------------------------------------------------------------
# Eigenray Shooting Solver
# -----------------------------------------------------------------------------

(defn find-eigenrays [profile z-src z-rcv target-range &opt tolerance num-rays]
  "Find acoustic eigenrays connecting source at (0, z_src) to receiver at (target_range, z_rcv)."
  (default tolerance 25.0) # 25 meters spatial arrival window
  (default num-rays 91)

  (def eigenrays @[])
  (def min-angle (* -35.0 c/deg-to-rad))
  (def max-angle (* 35.0 c/deg-to-rad))
  (def d-theta (/ (- max-angle min-angle) (- num-rays 1)))

  (for i 0 num-rays
    (def th (+ min-angle (* i d-theta)))
    (def ray (trace-acoustic-ray profile z-src th target-range 10.0))
    (def path (get ray :path))

    # Search for crossing of target range
    (for j 1 (length path)
      (def p0 (in path (- j 1)))
      (def p1 (in path j))
      (when (and (<= (get p0 :x) target-range) (>= (get p1 :x) target-range))
        (def dx (- (get p1 :x) (get p0 :x)))
        (def alpha (if (> dx 1e-6) (/ (- target-range (get p0 :x)) dx) 0.0))
        (def z-cross (+ (get p0 :z) (* alpha (- (get p1 :z) (get p0 :z)))))
        (def dz-err (math/abs (- z-cross z-rcv)))
        (when (<= dz-err tolerance)
          (def tau-cross (+ (get p0 :tau) (* alpha (- (get p1 :tau) (get p0 :tau)))))
          (array/push eigenrays {:launch-angle th
                                 :arrival-depth z-cross
                                 :travel-time tau-cross
                                 :surface-bounces (get ray :surface-bounces)
                                 :bottom-bounces (get ray :bottom-bounces)}))
        (break))))

  eigenrays)
