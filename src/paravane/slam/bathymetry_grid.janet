# src/paravane/slam/bathymetry_grid.janet
# 2.5D Digital Elevation Model (DEM) bathymetric grid with online Welford variance tracking.

(import ../core/constants :as c)
(import ../core/math :as m)

# -----------------------------------------------------------------------------
# Bathymetric Grid Representation
# -----------------------------------------------------------------------------

(defn make-bathymetry-grid [&opt cell-size-m]
  "Construct an empty 2.5D bathymetric elevation grid with spatial hashing."
  (default cell-size-m 2.0) # 2.0 meter spatial resolution
  @{:cell-size cell-size-m
    :cells @{}
    :min-x 1e9 :max-x -1e9
    :min-y 1e9 :max-y -1e9
    :total-soundings 0})

(defn coord-to-cell-indices [grid x y]
  (def cs (get grid :cell-size))
  [(math/floor (/ x cs))
   (math/floor (/ y cs))])

(defn cell-key [ix iy]
  (string ix ":" iy))

# -----------------------------------------------------------------------------
# Welford Online Depth and Variance Accumulation
# -----------------------------------------------------------------------------

(defn insert-sounding! [grid x y depth]
  "Insert a 3D seabed sounding into the grid, updating cell mean depth and sample variance."
  (def idxs (coord-to-cell-indices grid x y))
  (def ix (in idxs 0))
  (def iy (in idxs 1))
  (def key (cell-key ix iy))

  # Update grid bounding box
  (when (< x (get grid :min-x)) (put grid :min-x x))
  (when (> x (get grid :max-x)) (put grid :max-x x))
  (when (< y (get grid :min-y)) (put grid :min-y y))
  (when (> y (get grid :max-y)) (put grid :max-y y))
  (put grid :total-soundings (+ (get grid :total-soundings) 1))

  (def cells (get grid :cells))
  (def existing (get cells key))

  (if (nil? existing)
    # First sounding in this cell
    (put cells key @{:count 1
                     :mean-depth depth
                     :m2 0.0
                     :center-x (+ (* (+ ix 0.5) (get grid :cell-size)))
                     :center-y (+ (* (+ iy 0.5) (get grid :cell-size)))})
    (do
      # Welford online update algorithm
      (def count (+ (get existing :count) 1))
      (def delta (- depth (get existing :mean-depth)))
      (def next-mean (+ (get existing :mean-depth) (/ delta count)))
      (def delta2 (- depth next-mean))
      (def next-m2 (+ (get existing :m2) (* delta delta2)))

      (put existing :count count)
      (put existing :mean-depth next-mean)
      (put existing :m2 next-m2)))
  grid)

(defn insert-soundings-batch! [grid points-ned]
  "Insert an array of [north, east, depth] soundings into the bathymetric grid."
  (each pt points-ned
    (insert-sounding! grid (in pt 0) (in pt 1) (in pt 2)))
  grid)

# -----------------------------------------------------------------------------
# Grid Query and Depth Interpolation
# -----------------------------------------------------------------------------

(defn lookup-depth [grid x y]
  "Query estimated seabed depth at coordinates (x, y) with fallback."
  (def idxs (coord-to-cell-indices grid x y))
  (def key (cell-key (in idxs 0) (in idxs 1)))
  (def cell (get (get grid :cells) key))
  (if cell
    (get cell :mean-depth)
    nil))

(defn cell-variance [cell]
  "Compute sample variance of depth soundings within cell."
  (def count (get cell :count))
  (if (<= count 1)
    0.0
    (/ (get cell :m2) (- count 1))))

(defn grid-cell-count [grid]
  (length (keys (get grid :cells))))
