# test/test_core_math.janet
# Comprehensive unit tests for core vector, matrix, quaternion, and geodesy math.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/core/geodesy :as g)
(import ../src/paravane/core/stats :as s)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# Vector Math Tests
# -----------------------------------------------------------------------------

(def v1 [1.0 2.0 3.0])
(def v2 [4.0 5.0 6.0])

(h/assert-vec-near [5.0 7.0 9.0] (m/vec3-add v1 v2) 1e-9 "Vector addition")
(h/assert-vec-near [-3.0 -3.0 -3.0] (m/vec3-sub v1 v2) 1e-9 "Vector subtraction")
(h/assert-vec-near [2.0 4.0 6.0] (m/vec3-scale v1 2.0) 1e-9 "Vector scaling")
(h/assert-near 32.0 (m/vec3-dot v1 v2) 1e-9 "Vector dot product: 1*4 + 2*5 + 3*6 = 32")

# Cross product: [1, 2, 3] x [4, 5, 6] = [-3, 6, -3]
(h/assert-vec-near [-3.0 6.0 -3.0] (m/vec3-cross v1 v2) 1e-9 "Vector cross product")
(h/assert-near (math/sqrt 14.0) (m/vec3-norm v1) 1e-9 "Vector norm")
(h/assert-near 1.0 (m/vec3-norm (m/vec3-normalize v1)) 1e-9 "Vector normalization")

# Orthogonality of basis vectors
(h/assert-vec-near m/vec3-unit-z (m/vec3-cross m/vec3-unit-x m/vec3-unit-y) 1e-9 "X cross Y is Z")

# -----------------------------------------------------------------------------
# Matrix Math Tests
# -----------------------------------------------------------------------------

(def id m/mat3-identity)
(h/assert-vec-near v1 (m/mat3-mul-vec id v1) 1e-9 "Identity matrix vector multiplication")
(h/assert-near 3.0 (m/mat3-trace id) 1e-9 "Identity matrix trace is 3")
(h/assert-near 1.0 (m/mat3-det id) 1e-9 "Identity matrix determinant is 1")

# Skew-symmetric cross product matrix equivalence
(def skew-v1 (m/mat3-skew v1))
(def cross-prod (m/vec3-cross v1 v2))
(def mat-prod (m/mat3-mul-vec skew-v1 v2))
(h/assert-vec-near cross-prod mat-prod 1e-9 "Skew matrix cross product equivalence")

# Invertible matrix test
(def test-mat [2.0 1.0 0.0
               1.0 3.0 1.0
               0.0 1.0 2.0])
(def inv-mat (m/mat3-inv test-mat))
(h/assert-true (not (nil? inv-mat)) "Matrix inverse exists")
(def prod-mat (m/mat3-mul-mat test-mat inv-mat))
(h/assert-near 1.0 (in prod-mat 0) 1e-6 "M * M^-1 == I (0,0)")
(h/assert-near 0.0 (in prod-mat 1) 1e-6 "M * M^-1 == I (0,1)")
(h/assert-near 1.0 (in prod-mat 4) 1e-6 "M * M^-1 == I (1,1)")
(h/assert-near 1.0 (in prod-mat 8) 1e-6 "M * M^-1 == I (2,2)")

# Euler angle to rotation matrix round trip
(def phi 0.2)
(def theta -0.15)
(def psi 1.25)
(def r-mat (m/mat3-from-euler phi theta psi))
(def euler-rec (m/mat3-to-euler r-mat))
(h/assert-near phi (in euler-rec 0) 1e-6 "Euler round-trip roll")
(h/assert-near theta (in euler-rec 1) 1e-6 "Euler round-trip pitch")
(h/assert-near psi (in euler-rec 2) 1e-6 "Euler round-trip yaw")

# -----------------------------------------------------------------------------
# Quaternion Tests
# -----------------------------------------------------------------------------

(def q-id m/quat-identity)
(h/assert-near 1.0 (m/quat-norm q-id) 1e-9 "Quaternion identity norm")
(h/assert-vec-near v1 (m/quat-rot-vec q-id v1) 1e-9 "Quaternion identity rotation")

# 90-degree yaw rotation about Z axis
(def q-yaw90 (m/quat-from-euler 0.0 0.0 c/half-pi))
(def rot-x (m/quat-rot-vec q-yaw90 m/vec3-unit-x))
(h/assert-vec-near m/vec3-unit-y rot-x 1e-6 "Rotate [1,0,0] by 90-deg yaw yields [0,1,0]")

# Quaternion to/from Euler round trip
(def q-orig (m/quat-from-euler 0.1 -0.2 0.8))
(def q-angles (m/quat-to-euler q-orig))
(h/assert-near 0.1 (in q-angles 0) 1e-6 "Quat Euler roll")
(h/assert-near -0.2 (in q-angles 1) 1e-6 "Quat Euler pitch")
(h/assert-near 0.8 (in q-angles 2) 1e-6 "Quat Euler yaw")

# Quaternion integration test
(def omega [0.0 0.0 0.1]) # 0.1 rad/s yaw rate
(def q-next (m/quat-integrate q-id omega 1.0))
(h/assert-near 1.0 (m/quat-norm q-next) 1e-6 "Integrated quaternion remains unit length")

# -----------------------------------------------------------------------------
# Geodesy Tests (WGS-84 ECEF & NED)
# -----------------------------------------------------------------------------

# Reference location: Monterey Bay offshore survey site (Lat: 36.8 deg N, Lon: -122.0 deg W, Height: 0.0 m)
(def lat0 (* 36.8 c/deg-to-rad))
(def lon0 (* -122.0 c/deg-to-rad))
(def h0 0.0)
(def ref-origin [lat0 lon0 h0])

(def ecef0 (g/geodetic-to-ecef lat0 lon0 h0))
(def geodetic-rec (g/ecef-to-geodetic ecef0))

(h/assert-near lat0 (in geodetic-rec 0) 1e-7 "Geodetic to ECEF round-trip latitude")
(h/assert-near lon0 (in geodetic-rec 1) 1e-7 "Geodetic to ECEF round-trip longitude")
(h/assert-near h0 (in geodetic-rec 2) 1e-4 "Geodetic to ECEF round-trip height (sub-millimeter)")

# Displace 1000m North, 500m East, 100m Down
(def target-ned [1000.0 500.0 100.0])
(def target-geodetic (g/ned-to-geodetic target-ned ref-origin))
(def recovered-ned (g/geodetic-to-ned (in target-geodetic 0) (in target-geodetic 1) (in target-geodetic 2) ref-origin))

(h/assert-near (in target-ned 0) (in recovered-ned 0) 1e-3 "NED round-trip North")
(h/assert-near (in target-ned 1) (in recovered-ned 1) 1e-3 "NED round-trip East")
(h/assert-near (in target-ned 2) (in recovered-ned 2) 1e-3 "NED round-trip Down")

# Great-circle distance test
(def dist (g/haversine-distance lat0 lon0 lat0 (+ lon0 (* 0.01 c/deg-to-rad))))
(h/assert-true (> dist 800.0) "Haversine longitudinal distance check")
(h/assert-true (< dist 1000.0) "Haversine distance within bounds")

# -----------------------------------------------------------------------------
# Statistics and Cholesky Tests
# -----------------------------------------------------------------------------

(def sym-mat [4.0 12.0 -16.0
              12.0 37.0 -43.0
              -16.0 -43.0 98.0])
(def l-chol (s/cholesky-3x3 sym-mat))
(h/assert-true (not (nil? l-chol)) "Cholesky decomposition succeeded")
(h/assert-near 2.0 (in l-chol 0) 1e-6 "Cholesky L(0,0) = sqrt(4) = 2")
(h/assert-near 6.0 (in l-chol 3) 1e-6 "Cholesky L(1,0) = 12/2 = 6")

(def gate-ok (s/check-chi2-gate-3d [0.1 0.1 0.1] (m/mat3-diag 1.0 1.0 1.0)))
(h/assert-true gate-ok "Chi-square gating on small residual passes")

(def gate-reject (s/check-chi2-gate-3d [10.0 10.0 10.0] (m/mat3-diag 1.0 1.0 1.0)))
(h/assert-false gate-reject "Chi-square gating on huge residual rejects outlier")

(h/summarize-tests "Core Math, Geodesy, and Statistics")
