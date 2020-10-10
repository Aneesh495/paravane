# test/test_ocean_physics.janet
# Comprehensive test suite for ocean sound speed, thermodynamics, and current dynamics.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/ocean/sound_speed :as ss)
(import ../src/paravane/ocean/equation_of_state :as eos)
(import ../src/paravane/ocean/profile :as p)
(import ../src/paravane/ocean/currents :as cur)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# Sound Speed Formulation Benchmarks
# -----------------------------------------------------------------------------

# Test Case 1: Standard oceanic surface condition (T=15 C, S=35 PSU, D=0 m)
(def c-surf-mack (ss/sound-speed-mackenzie 15.0 35.0 0.0))
# Benchmark: Mackenzie gives 1506.69 m/s at 15 C, 35 PSU
(h/assert-near 1506.69 c-surf-mack 0.05 "Mackenzie surface sound speed at 15 C, 35 PSU")

# Test Case 2: Deep abyssal condition (T=2.0 C, S=34.8 PSU, D=4000 m)
(def c-deep-mack (ss/sound-speed-mackenzie 2.0 34.8 4000.0))
# Depth increases sound speed by ~16.3 m/s per 1000m despite cold temperatures
(h/assert-true (> c-deep-mack 1520.0) "Deep ocean pressure increases sound speed above 1520 m/s")
(h/assert-true (< c-deep-mack 1530.0) "Deep ocean sound speed within expected bounds")

# Test Case 3: Surface formulation agreement between Del Grosso, UNESCO, and Mackenzie
(def c-del-grosso-surf (ss/sound-speed-del-grosso 10.0 35.0 0.0))
(def c-unesco-surf (ss/sound-speed-unesco 10.0 35.0 0.0))
(def c-mack-surf (ss/sound-speed-mackenzie 10.0 35.0 0.0))
# Surface formulations agree within 0.4 m/s
(h/assert-near c-del-grosso-surf c-unesco-surf 0.45 "Del Grosso and UNESCO surface speeds agree closely")
(h/assert-near c-mack-surf c-unesco-surf 0.10 "Mackenzie and UNESCO surface speeds agree within 0.1 m/s")

# Test Case 4: Deep water pressure agreement between UNESCO (100 bar) and Mackenzie (1000m)
(def c-unesco-100bar (ss/sound-speed-unesco 10.0 35.0 100.0))
(def c-mack-1000m (ss/sound-speed-mackenzie 10.0 35.0 1000.0))
(h/assert-near c-mack-1000m c-unesco-100bar 0.15 "Mackenzie and UNESCO agree within 0.15 m/s at 1000m / 100bar")

# Test Case 5: Coppens shallow water approximation vs Mackenzie
(def c-coppens (ss/sound-speed-coppens 12.0 35.0 200.0))
(def c-mack-200 (ss/sound-speed-mackenzie 12.0 35.0 200.0))
(h/assert-near c-mack-200 c-coppens 0.5 "Coppens vs Mackenzie within 0.5 m/s in shallow water")

# Test Case 6: Vertical sound speed gradient
(def grad (ss/sound-speed-gradient 10.0 35.0 1000.0))
# In deep water, thermal gradient is small, and pressure gradient is ~0.016 (m/s)/m
(h/assert-near 0.016 grad 0.003 "Vertical sound speed gradient near 0.016 s^-1 due to hydrostatic pressure")

# -----------------------------------------------------------------------------
# Seawater Thermodynamics & Saunders Conversion
# -----------------------------------------------------------------------------

# Saunders pressure to depth: 1000 dbar at 45 deg latitude
(def lat45 (* 45.0 c/deg-to-rad))
(def depth-1000 (eos/pressure-to-depth-saunders 1000.0 lat45))
# 1000 dbar corresponds to approximately 989.25 meters depth
(h/assert-near 989.25 depth-1000 0.5 "1000 dbar pressure corresponds to 989.25 meters via Saunders (1981)")

# Depth to pressure approximation inversion
(def p-approx (eos/depth-to-pressure-approx depth-1000 lat45))
(h/assert-near 1000.0 p-approx 5.0 "Depth to pressure approximation inverts Saunders within 0.5 percent")

# Seawater density IES 80 at T=10 C, S=35 PSU, P=0 bar
(def rho-surf (eos/seawater-density-ies80 10.0 35.0 0.0))
(h/assert-near 1026.97 rho-surf 0.1 "Standard seawater density at 10 C is ~1026.97 kg/m^3")

# Sigma-t density anomaly
(def sigma-t (eos/density-anomaly-sigma-t 10.0 35.0))
(h/assert-near 26.97 sigma-t 0.1 "Sigma-t density anomaly is ~26.97 kg/m^3")

# -----------------------------------------------------------------------------
# Sound Velocity Profile (SVP) & Cubic Spline Interpolation
# -----------------------------------------------------------------------------

(def depths [0.0 50.0 150.0 400.0 800.0 1200.0 2000.0])
(def speeds [1510.0 1508.0 1485.0 1475.0 1480.0 1492.0 1515.0])
(def test-svp (p/make-svp depths speeds))

(def spline (p/build-natural-cubic-spline depths speeds))
# Spline must interpolate knots exactly
(for i 0 (length depths)
  (def res (p/eval-cubic-spline spline (in depths i)))
  (h/assert-near (in speeds i) (get res :val) 1e-6 (string "Spline exact interpolation at knot " i)))

# Intermediate smooth interpolation test
(def mid-res (p/eval-cubic-spline spline 100.0))
(h/assert-true (and (< (get mid-res :val) 1508.0) (> (get mid-res :val) 1485.0))
               "Spline provides smooth monotonic transition in thermocline")

# Linear interpolation comparison
(def lin-res (p/svp-eval-linear test-svp 100.0))
(h/assert-near 1496.5 (get lin-res :speed) 1e-3 "Linear interpolation at midpoint between 50m and 150m")

# Munk canonical sound channel profile test
(def munk-svp (p/munk-canonical-profile 1000.0 1490.0 0.00737 1300.0 4000.0 41))
(def sofar (p/find-sofar-axis munk-svp))
(h/assert-near 1000.0 (get sofar :depth) 1e-3 "SOFAR channel axis accurately identified at 1000m")
(h/assert-near 1490.0 (get sofar :min-speed) 1e-3 "SOFAR channel minimum sound speed matches axis speed")

# -----------------------------------------------------------------------------
# Ocean Currents & Ekman Transport
# -----------------------------------------------------------------------------

# Coriolis parameter at 45 deg N
(def f45 (cur/coriolis-f lat45))
(h/assert-near 1.031e-4 f45 1e-6 "Coriolis parameter at 45 deg N is ~1.03e-4 rad/s")

# Ekman layer depth with 10 m/s wind
(def de (cur/ekman-layer-depth 10.0 lat45))
(h/assert-true (> de 50.0) "Ekman layer depth > 50 meters")
(h/assert-true (< de 150.0) "Ekman layer depth < 150 meters")

# Total current composite query
(def cur-model (cur/make-current-model 0.15 -0.10 8.0 0.0 0.20 0.05))
(def cur-vec (cur/current-at-depth cur-model 20.0 1000.0 lat45))
(h/assert-equal 3 (length cur-vec) "Current vector has 3 spatial dimensions")
(h/assert-true (> (math/sqrt (+ (* (in cur-vec 0) (in cur-vec 0)) (* (in cur-vec 1) (in cur-vec 1)))) 0.05)
               "Total current magnitude is non-zero and physically realistic")

(h/summarize-tests "Oceanographic Physics and Thermodynamics")
