# src/paravane/slam/terrain_matching.janet
# Terrain-Aided Navigation (TAN) particle filter map-matching against bathymetric reference grids.

(import ../core/constants :as c)
(import ../core/math :as m)
(import ../core/stats :as s)
(import ./bathymetry_grid :as bg)

# -----------------------------------------------------------------------------
# Particle Representation
# -----------------------------------------------------------------------------

(defn make-particle [x y &opt weight]
  (default weight 1.0)
  @{:x x :y y :weight weight})

(defn make-tan-particle-filter [initial-pos-xy num-particles init-spread-m]
  "Initialize Terrain-Aided Navigation particle filter centered at estimated position."
  (def particles @[])
  (def init-w (/ 1.0 num-particles))
  (def x0 (in initial-pos-xy 0))
  (def y0 (in initial-pos-xy 1))

  (for i 0 num-particles
    (def px (+ x0 (s/rand-gaussian 0.0 init-spread-m)))
    (def py (+ y0 (s/rand-gaussian 0.0 init-spread-m)))
    (array/push particles (make-particle px py init-w)))

  @{:particles particles
    :num-particles num-particles
    :est-pos [x0 y0]})

# -----------------------------------------------------------------------------
# Particle Motion Update (DVL / INS Dead Reckoning)
# -----------------------------------------------------------------------------

(defn tan-predict! [pf delta-pos-xy &opt motion-noise-m]
  "Propagate particles forward by dead-reckoning displacement with Gaussian drift."
  (default motion-noise-m 0.20) # 20 cm motion uncertainty per step
  (def dx (in delta-pos-xy 0))
  (def dy (in delta-pos-xy 1))
  (def particles (get pf :particles))

  (each p particles
    (def noise-x (s/rand-gaussian 0.0 motion-noise-m))
    (def noise-y (s/rand-gaussian 0.0 motion-noise-m))
    (put p :x (+ (get p :x) dx noise-x))
    (put p :y (+ (get p :y) dy noise-y)))

  (def cur-est (get pf :est-pos))
  (put pf :est-pos [(+ (in cur-est 0) dx) (+ (in cur-est 1) dy)])
  pf)

# -----------------------------------------------------------------------------
# Particle Measurement Update (Bathymetric Depth Likelihood)
# -----------------------------------------------------------------------------

(defn tan-update! [pf bathy-grid measured-depth &opt depth-noise-m]
  "Weight particles by likelihood of observed seabed depth against reference bathymetric grid."
  (default depth-noise-m 0.50) # 50 cm sounding precision
  (def var2 (* 2.0 depth-noise-m depth-noise-m))
  (def particles (get pf :particles))

  (var total-weight 0.0)

  (each p particles
    (def map-z (bg/lookup-depth bathy-grid (get p :x) (get p :y)))
    (if (nil? map-z)
      # Particle outside mapped grid: assign small uniform likelihood
      (put p :weight (* (get p :weight) 0.001))
      (let [err (- measured-depth map-z)
            likelihood (math/exp (/ (* (- err) err) var2))]
        (put p :weight (* (get p :weight) (max 1e-12 likelihood)))))
    (+= total-weight (get p :weight)))

  # Normalize particle weights
  (if (> total-weight 1e-15)
    (each p particles
      (put p :weight (/ (get p :weight) total-weight)))
    (let [uniform-w (/ 1.0 (length particles))]
      (each p particles (put p :weight uniform-w))))

  # Compute weighted mean position estimate
  (var mean-x 0.0)
  (var mean-y 0.0)
  (each p particles
    (def w (get p :weight))
    (+= mean-x (* (get p :x) w))
    (+= mean-y (* (get p :y) w)))

  (put pf :est-pos [mean-x mean-y])
  pf)

# -----------------------------------------------------------------------------
# Low-Variance Systematic Resampling
# -----------------------------------------------------------------------------

(defn effective-particle-count [pf]
  (def particles (get pf :particles))
  (var sum-w2 0.0)
  (each p particles
    (def w (get p :weight))
    (+= sum-w2 (* w w)))
  (if (> sum-w2 0.0) (/ 1.0 sum-w2) 0.0))

(defn tan-resample! [pf]
  "Perform low-variance systematic resampling when effective particle count drops below N/2."
  (def n (get pf :num-particles))
  (def n-eff (effective-particle-count pf))

  (when (< n-eff (/ n 2.0))
    (def old-particles (get pf :particles))
    (def new-particles (array/new n))
    (def step (/ 1.0 n))
    (var r (* (math/random) step))
    (var c (get (in old-particles 0) :weight))
    (var i 0)

    (for m 0 n
      (def u (+ r (* m step)))
      (while (> u c)
        (++ i)
        (+= c (get (in old-particles i) :weight)))
      (def chosen (in old-particles i))
      (array/push new-particles (make-particle (get chosen :x) (get chosen :y) step)))

    (put pf :particles new-particles))
  pf)

(def predict! tan-predict!)
(def update! tan-update!)
(def resample! tan-resample!)
(def effective-count effective-particle-count)
