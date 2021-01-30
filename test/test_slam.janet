# test/test_slam.janet
# Comprehensive unit tests for Multibeam Echo-Sounder, bathymetric DEM grids, and terrain SLAM.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/slam/mbes :as mbes)
(import ../src/paravane/slam/bathymetry_grid :as bg)
(import ../src/paravane/slam/terrain_matching :as tan)
(import ../src/paravane/slam/point_cloud :as pc)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# Multibeam Echo-Sounder (MBES) Tests
# -----------------------------------------------------------------------------

(def mbes-cfg (mbes/make-mbes-config 64 120.0 5.0 [0.20 0.0 0.10]))
(def angles (get mbes-cfg :beam-angles))

# 64 beams spanning 120 degrees (-60 deg port to +60 deg starboard)
(h/assert-equal 64 (length angles) "MBES configuration creates exactly 64 beams")
(h/assert-near (* -60.0 c/deg-to-rad) (in angles 0) 1e-4 "First beam points 60 deg to port")
(h/assert-near (* 60.0 c/deg-to-rad) (in angles 63) 1e-4 "Last beam points 60 deg to starboard")

# Project single nadir beam at range 20m: striking flat bottom at 20.1m depth
# Beam index 31 or 32 is near vertical
(def pt-body (mbes/project-beam-sounding-body mbes-cfg 31 20.0))
(h/assert-near 0.20 (in pt-body 0) 1e-3 "Nadir beam x-offset matches transducer mounting x-offset")
(h/assert-near 20.10 (in pt-body 2) 0.5 "Nadir beam z-offset matches 20m range + mounting height")

# Project synthetic swath to NED from vehicle at [100, 200, 50]
(def ranges @[])
(for i 0 64 (array/push ranges 25.0)) # 25m uniform range returns
(def swath-ned (mbes/project-swath-to-ned mbes-cfg [100.0 200.0 50.0] m/mat3-identity ranges))
(h/assert-equal 64 (length swath-ned) "Full swath produces 64 3D points in NED frame")

# -----------------------------------------------------------------------------
# Bathymetry Grid & Welford Variance Tests
# -----------------------------------------------------------------------------

(def grid (bg/make-bathymetry-grid 2.0)) # 2.0m resolution

# Insert soundings into cell (10, 20) -> x in [20, 22), y in [40, 42)
(bg/insert-sounding! grid 21.0 41.0 100.0)
(bg/insert-sounding! grid 21.2 41.4 102.0)
(bg/insert-sounding! grid 20.8 40.8 104.0)

(h/assert-equal 1 (bg/grid-cell-count grid) "All three soundings fall into single 2m grid cell")
(def depth-est (bg/lookup-depth grid 21.0 41.0))
(h/assert-near 102.0 depth-est 1e-6 "Mean depth calculated via Welford algorithm is exactly 102.0m")

(def cell (get (get grid :cells) (bg/cell-key 10 20)))
(def var-est (bg/cell-variance cell))
# Sample variance of {100, 102, 104} is 4.0
(h/assert-near 4.0 var-est 1e-6 "Sample depth variance is exactly 4.0 m^2")

# Query unmapped coordinate
(h/assert-true (nil? (bg/lookup-depth grid 500.0 500.0)) "Querying unmapped cell returns nil")

# -----------------------------------------------------------------------------
# Terrain-Aided Navigation (TAN) Particle Filter Tests
# -----------------------------------------------------------------------------

(def pf (tan/make-tan-particle-filter [100.0 200.0] 50 5.0))
(h/assert-equal 50 (length (get pf :particles)) "Particle filter initializes with 50 particles")

# Predict step: vehicle moves +2.0m North, +1.0m East
(tan/predict! pf [2.0 1.0] 0.05)
(def est-pos-pred (get pf :est-pos))
(h/assert-near 102.0 (in est-pos-pred 0) 1.5 "Dead reckoning advances particle cloud North by ~2m")

# Update step against bathymetric grid
# Populate grid with depth gradient: depth = 100 + 0.1 * x
(for gx 90 120
  (for gy 190 220
    (bg/insert-sounding! grid gx gy (+ 100.0 (* 0.1 gx)))))

(tan/update! pf grid 110.2 0.3)
# Estimated position should converge near x = (110.2 - 100) / 0.1 = 102m
(def est-pos-upd (get pf :est-pos))
(h/assert-near 102.0 (in est-pos-upd 0) 2.5 "TAN particle filter weights align estimate with bathymetric depth")

# Resample test
(def n-eff (tan/effective-particle-count pf))
(h/assert-true (> n-eff 0.0) "Effective particle count is positive")
(tan/resample! pf)
(h/assert-equal 50 (length (get pf :particles)) "Resampling preserves particle count at 50")

# -----------------------------------------------------------------------------
# 3D Point Cloud & Iterative Closest Point (ICP) Tests
# -----------------------------------------------------------------------------

# Construct synthetic reference point cloud (planar seafloor patch with mound)
(def target-cloud @[[0.0 0.0 100.0]
                    [10.0 0.0 100.0]
                    [0.0 10.0 100.0]
                    [10.0 10.0 102.0]
                    [5.0 5.0 105.0]])

# Source cloud shifted by dx=0.5m, dy=0.2m
(def source-cloud @[])
(each pt target-cloud
  (array/push source-cloud [(+ (in pt 0) 0.50) (+ (in pt 1) 0.20) (in pt 2)]))

(def centroid-tgt (pc/point-cloud-centroid target-cloud))
(h/assert-near 5.0 (in centroid-tgt 0) 1e-6 "Target cloud X-centroid is 5.0m")
(h/assert-near 5.0 (in centroid-tgt 1) 1e-6 "Target cloud Y-centroid is 5.0m")

# Run ICP registration
(def icp-res (pc/run-icp-registration source-cloud target-cloud 10 1e-4))
(h/assert-true (get icp-res :converged) "ICP point cloud registration converged successfully")
(h/assert-near -0.50 (in (get icp-res :t) 0) 0.10 "ICP recovered translational X-offset of -0.50m")
(h/assert-near -0.20 (in (get icp-res :t) 1) 0.10 "ICP recovered translational Y-offset of -0.20m")

(h/summarize-tests "Multibeam Sonar, Bathymetric DEM Grids, and Point Cloud SLAM")
