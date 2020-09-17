# src/paravane/core/stats.janet
# Statistical utilities, covariance management, and chi-square gating for Kalman filtering.

(import ./math :as m)

(var *has-spare-gaussian* false)
(var *spare-gaussian* 0.0)

(defn rand-gaussian [&opt mean stddev]
  "Generate normally distributed random variable using Box-Muller transform."
  (default mean 0.0)
  (default stddev 1.0)
  (if *has-spare-gaussian*
    (do
      (set *has-spare-gaussian* false)
      (+ mean (* stddev *spare-gaussian*)))
    (do
      (var u (math/random))
      (while (<= u 1e-15) (set u (math/random)))
      (def v (math/random))
      (def r (math/sqrt (* -2.0 (math/log u))))
      (def theta (* 2.0 math/pi v))
      (set *spare-gaussian* (* r (math/sin theta)))
      (set *has-spare-gaussian* true)
      (+ mean (* stddev (* r (math/cos theta)))))))

(defn rand-gaussian-vec3 [&opt mean-v stddev-v]
  (default mean-v [0.0 0.0 0.0])
  (default stddev-v [1.0 1.0 1.0])
  [(rand-gaussian (in mean-v 0) (in stddev-v 0))
   (rand-gaussian (in mean-v 1) (in stddev-v 1))
   (rand-gaussian (in mean-v 2) (in stddev-v 2))])

# -----------------------------------------------------------------------------
# Chi-Square Gating Thresholds (p = 0.01 and p = 0.05)
# -----------------------------------------------------------------------------

(def chi2-table-99
  # Degrees of freedom: 1 to 6
  {1 6.635
   2 9.210
   3 11.345
   4 13.277
   5 15.086
   6 16.812})

(def chi2-table-95
  {1 3.841
   2 5.991
   3 7.815
   4 9.488
   5 11.070
   6 12.592})

(defn chi2-threshold [dof &opt p-val]
  (default p-val 0.01)
  (if (<= p-val 0.01)
    (get chi2-table-99 dof 15.0)
    (get chi2-table-95 dof 10.0)))

(defn mahalanobis-dist2-1d [residual variance]
  "Compute squared Mahalanobis distance for 1D scalar residual."
  (if (<= variance 1e-15)
    1e12
    (/ (* residual residual) variance)))

(defn mahalanobis-dist2-3d [residual-vec3 cov-mat3]
  "Compute squared Mahalanobis distance r^T * Cov^-1 * r for 3D vector residual."
  (def inv-cov (m/mat3-inv cov-mat3))
  (if (nil? inv-cov)
    1e12
    (let [temp (m/mat3-mul-vec inv-cov residual-vec3)]
      (m/vec3-dot residual-vec3 temp))))

(defn check-chi2-gate-3d [residual-vec3 cov-mat3 &opt dof p-val]
  "Perform chi-square residual gating validation. Returns true if measurement is statistically consistent."
  (default dof 3)
  (default p-val 0.01)
  (def d2 (mahalanobis-dist2-3d residual-vec3 cov-mat3))
  (def thresh (chi2-threshold dof p-val))
  (<= d2 thresh))

(defn cholesky-3x3 [m-mat]
  "Compute lower triangular Cholesky factor L such that L * L^T = M for symmetric positive-definite 3x3 matrix."
  (def a00 (in m-mat 0)) (def a01 (in m-mat 1)) (def a02 (in m-mat 2))
  (def a10 (in m-mat 3)) (def a11 (in m-mat 4)) (def a12 (in m-mat 5))
  (def a20 (in m-mat 6)) (def a21 (in m-mat 7)) (def a22 (in m-mat 8))

  (if (<= a00 0.0) (break nil))
  (def l00 (math/sqrt a00))
  (def l10 (/ a10 l00))
  (def l20 (/ a20 l00))

  (def d11 (- a11 (* l10 l10)))
  (if (<= d11 0.0) (break nil))
  (def l11 (math/sqrt d11))
  (def l21 (/ (- a21 (* l20 l10)) l11))

  (def d22 (- a22 (* l20 l20) (* l21 l21)))
  (if (<= d22 0.0) (break nil))
  (def l22 (math/sqrt d22))

  [l00 0.0 0.0
   l10 l11 0.0
   l20 l21 l22])
