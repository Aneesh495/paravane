# src/paravane/acoustics/lbl.janet
# Long Baseline (LBL) acoustic seabed transponder array multilateration solver.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# Transponder Beacon Representation
# -----------------------------------------------------------------------------

(defn make-transponder [id-tag pos-ned turnaround-delay-s]
  {:id id-tag
   :pos pos-ned # [north, east, down] in meters
   :turnaround turnaround-delay-s})

(defn make-lbl-array [transponders]
  {:beacons transponders
   :count (length transponders)})

# -----------------------------------------------------------------------------
# Range Calculation from Acoustic Round-Trip Time
# -----------------------------------------------------------------------------

(defn twtt-to-one-way-range [twtt-seconds turnaround-delay-s sound-speed]
  "Convert round-trip acoustic travel time to one-way acoustic range."
  (def net-time (max 0.0 (- twtt-seconds turnaround-delay-s)))
  (* 0.5 net-time sound-speed))

# -----------------------------------------------------------------------------
# Gauss-Newton Spherical Multilateration Solver
# -----------------------------------------------------------------------------

(defn solve-lbl-position-gauss-step [beacons measured-ranges x-est]
  "Perform one Gauss-Newton iteration step for 3D multilateration."
  (def m-count (length beacons))
  (var jtj00 0.0) (var jtj01 0.0) (var jtj02 0.0)
  (var jtj10 0.0) (var jtj11 0.0) (var jtj12 0.0)
  (var jtj20 0.0) (var jtj21 0.0) (var jtj22 0.0)

  (var jtr0 0.0)
  (var jtr1 0.0)
  (var jtr2 0.0)

  (for i 0 m-count
    (def b-pos (get (in beacons i) :pos))
    (def d-vec (m/vec3-sub x-est b-pos))
    (def dist (max 1e-6 (m/vec3-norm d-vec))))

  (for i 0 m-count
    (def b-pos (get (in beacons i) :pos))
    (def d-vec (m/vec3-sub x-est b-pos))
    (def dist (max 1e-6 (m/vec3-norm d-vec)))
    (def u (/ (in d-vec 0) dist))
    (def v (/ (in d-vec 1) dist))
    (def w (/ (in d-vec 2) dist))

    (def meas-r (in measured-ranges i))
    (def residual (- dist meas-r))

    # Accumulate J^T * J
    (+= jtj00 (* u u)) (+= jtj01 (* u v)) (+= jtj02 (* u w))
    (+= jtj10 (* v u)) (+= jtj11 (* v v)) (+= jtj12 (* v w))
    (+= jtj20 (* w u)) (+= jtj21 (* w v)) (+= jtj22 (* w w))

    # Accumulate J^T * residual
    (+= jtr0 (* u residual))
    (+= jtr1 (* v residual))
    (+= jtr2 (* w residual)))

  (def jtj [jtj00 jtj01 jtj02
            jtj10 jtj11 jtj12
            jtj20 jtj21 jtj22])

  (def inv-jtj (m/mat3-inv jtj))
  (if (nil? inv-jtj)
    {:delta [0.0 0.0 0.0] :cov nil :singular true}
    (let [delta (m/mat3-mul-vec inv-jtj [jtr0 jtr1 jtr2])]
      {:delta delta :cov inv-jtj :singular false})))

(defn solve-lbl-position [beacons measured-ranges initial-guess &opt max-iters tol]
  "Solve for 3D AUV position using iterative Gauss-Newton multilateration."
  (default max-iters 20)
  (default tol 1e-3)

  (var cur-pos initial-guess)
  (var converged false)
  (var last-cov nil)

  (for iter 0 max-iters
    (def step-res (solve-lbl-position-gauss-step beacons measured-ranges cur-pos))
    (when (get step-res :singular)
      (break))
    (def delta (get step-res :delta))
    (set last-cov (get step-res :cov))
    (set cur-pos (m/vec3-sub cur-pos delta))
    (when (< (m/vec3-norm delta) tol)
      (set converged true)
      (break)))

  {:pos cur-pos
   :converged converged
   :cov last-cov})

# -----------------------------------------------------------------------------
# Dilution of Precision (DOP) Metrics
# -----------------------------------------------------------------------------

(defn compute-lbl-dop [cov-mat3]
  "Compute Geometric, Horizontal, and Vertical Dilution of Precision from covariance."
  (if (nil? cov-mat3)
    {:gdop 99.9 :hdop 99.9 :vdop 99.9}
    (let [var-n (in cov-mat3 0)
          var-e (in cov-mat3 4)
          var-d (in cov-mat3 8)
          hdop (math/sqrt (max 0.0 (+ var-n var-e)))
          vdop (math/sqrt (max 0.0 var-d))
          gdop (math/sqrt (max 0.0 (+ var-n var-e var-d)))]
      {:gdop gdop :hdop hdop :vdop vdop})))
