# src/paravane/slam/point_cloud.janet
# 3D point cloud downsampling, nearest neighbor correspondence, and Iterative Closest Point (ICP).

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# Centroid and Demeaning
# -----------------------------------------------------------------------------

(defn point-cloud-centroid [points]
  "Compute 3D mean center of point cloud."
  (def n (length points))
  (if (zero? n)
    [0.0 0.0 0.0]
    (let [sum-v (array/new 3)]
      (array/push sum-v 0.0) (array/push sum-v 0.0) (array/push sum-v 0.0)
      (each p points
        (put sum-v 0 (+ (in sum-v 0) (in p 0)))
        (put sum-v 1 (+ (in sum-v 1) (in p 1)))
        (put sum-v 2 (+ (in sum-v 2) (in p 2))))
      [(/ (in sum-v 0) n)
       (/ (in sum-v 1) n)
       (/ (in sum-v 2) n)])))

# -----------------------------------------------------------------------------
# Nearest Neighbor Correspondence Search
# -----------------------------------------------------------------------------

(defn find-nearest-neighbor [target-points query-point]
  "Locate closest point in target cloud to query point via Euclidean distance."
  (var min-d2 1e12)
  (var best-pt nil)

  (each tp target-points
    (def dx (- (in tp 0) (in query-point 0)))
    (def dy (- (in tp 1) (in query-point 1)))
    (def dz (- (in tp 2) (in query-point 2)))
    (def d2 (+ (* dx dx) (* dy dy) (* dz dz)))
    (when (< d2 min-d2)
      (set min-d2 d2)
      (set best-pt tp)))

  {:point best-pt :dist2 min-d2})

# -----------------------------------------------------------------------------
# Arun Rigid Transformation Solver (Cross-Covariance)
# -----------------------------------------------------------------------------

(defn estimate-rigid-transform-arun [source-points target-points]
  "Estimate optimal 3D translation and rotation aligning paired point correspondences."
  (def n (min (length source-points) (length target-points)))
  (if (< n 3)
    {:r m/mat3-identity :t [0.0 0.0 0.0] :valid false}
    (do
      (def c-src (point-cloud-centroid source-points))
      (def c-tgt (point-cloud-centroid target-points))

      # Compute 3x3 cross-covariance matrix H = sum( (src_i - c_src) * (tgt_i - c_tgt)^T )
      (var h00 0.0) (var h01 0.0) (var h02 0.0)
      (var h10 0.0) (var h11 0.0) (var h12 0.0)
      (var h20 0.0) (var h21 0.0) (var h22 0.0)

      (for i 0 n
        (def ps (m/vec3-sub (in source-points i) c-src))
        (def pt (m/vec3-sub (in target-points i) c-tgt))

        (+= h00 (* (in ps 0) (in pt 0))) (+= h01 (* (in ps 0) (in pt 1))) (+= h02 (* (in ps 0) (in pt 2)))
        (+= h10 (* (in ps 1) (in pt 0))) (+= h11 (* (in ps 1) (in pt 1))) (+= h12 (* (in ps 1) (in pt 2)))
        (+= h20 (* (in ps 2) (in pt 0))) (+= h21 (* (in ps 2) (in pt 1))) (+= h22 (* (in ps 2) (in pt 2))))

      # Small angle approximation for rotation update in subsea scan matching
      (def rot-z (/ (- h01 h10) (max 1e-6 (+ h00 h11))))
      (def psi-opt (math/atan rot-z))
      (def r-mat (m/mat3-from-euler 0.0 0.0 psi-opt))

      # Translation: t = c_tgt - R * c_src
      (def r-c-src (m/mat3-mul-vec r-mat c-src))
      (def t-vec (m/vec3-sub c-tgt r-c-src))

      {:r r-mat :t t-vec :valid true})))

# -----------------------------------------------------------------------------
# Iterative Closest Point (ICP) Loop
# -----------------------------------------------------------------------------

(defn run-icp-registration [source-cloud target-cloud &opt max-iters tol]
  "Register source point cloud to target bathymetric point cloud using Iterative Closest Point."
  (default max-iters 15)
  (default tol 1e-4)

  (var cur-cloud source-cloud)
  (var total-r m/mat3-identity)
  (var total-t [0.0 0.0 0.0])
  (var converged false)
  (var prev-error 1e12)

  (for iter 0 max-iters
    # 1. Establish correspondences
    (def paired-targets @[])
    (var total-dist2 0.0)

    (each sp cur-cloud
      (def best-match (find-nearest-neighbor target-cloud sp))
      (array/push paired-targets (get best-match :point))
      (+= total-dist2 (get best-match :dist2)))

    (def mean-err (/ total-dist2 (max 1 (length cur-cloud))))

    # Check convergence
    (if (< (math/abs (- prev-error mean-err)) tol)
      (do
        (set converged true)
        (break))
      (set prev-error mean-err))

    # 2. Estimate transformation
    (def step-tf (estimate-rigid-transform-arun cur-cloud paired-targets))
    (def step-r (get step-tf :r))
    (def step-t (get step-tf :t))

    # Accumulate composite transformation
    (set total-r (m/mat3-mul-mat step-r total-r))
    (set total-t (m/vec3-add (m/mat3-mul-vec step-r total-t) step-t))

    # 3. Transform source cloud
    (def next-cloud @[])
    (each sp cur-cloud
      (def p-rot (m/mat3-mul-vec step-r sp))
      (array/push next-cloud (m/vec3-add p-rot step-t)))
    (set cur-cloud next-cloud))

  {:registered-cloud cur-cloud
   :r total-r
   :t total-t
   :mean-error prev-error
   :converged converged})
