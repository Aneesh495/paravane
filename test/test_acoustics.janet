# test/test_acoustics.janet
# Comprehensive unit test suite for acoustic ray tracing, transmission loss, USBL, and LBL.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/ocean/profile :as prof)
(import ../src/paravane/acoustics/ray_tracer :as rt)
(import ../src/paravane/acoustics/transmission_loss :as tl)
(import ../src/paravane/acoustics/usbl :as usbl)
(import ../src/paravane/acoustics/lbl :as lbl)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# Ray Tracing and Snell's Law Tests
# -----------------------------------------------------------------------------

# Construct a test sound velocity profile with a negative thermocline gradient
(def depths [0.0 50.0 200.0 500.0 1000.0 2000.0])
(def speeds [1510.0 1510.0 1480.0 1470.0 1485.0 1510.0])
(def test-svp (prof/make-svp depths speeds))

# Ray launched at 100m depth with -5 degrees pitch (slight upward angle)
(def theta0 (* -5.0 c/deg-to-rad))
(def ray (rt/trace-acoustic-ray test-svp 100.0 theta0 2000.0))
(def path (get ray :path))

(h/assert-true (> (length path) 2) "Ray path contains multiple integration steps")
(h/assert-true (>= (get ray :final-x) 2000.0) "Ray successfully reaches target range")
(h/assert-true (> (get ray :travel-time) 1.0) "Ray travel time is physically realistic (> 1.0 s for 2km)")

# Eigenray shooting test: find ray arriving at 500m depth at 2500m range
(def eigenrays (rt/find-eigenrays test-svp 100.0 500.0 2500.0 10.0 61))
(h/assert-true (> (length eigenrays) 0) "Eigenray shooting finds valid acoustic propagation path")

# -----------------------------------------------------------------------------
# Acoustic Transmission Loss & Wenz Ambient Noise Tests
# -----------------------------------------------------------------------------

# Thorp attenuation at 25 kHz
(def alpha-25 (tl/thorp-attenuation-db-per-km 25.0))
# Benchmark: at 25 kHz, alpha is typically ~5 to 6 dB/km
(h/assert-true (> alpha-25 4.0) "Thorp attenuation at 25 kHz > 4 dB/km")
(h/assert-true (< alpha-25 7.0) "Thorp attenuation at 25 kHz < 7 dB/km")

# Spherical spreading loss at 1000m: 20 * log10(1000) = 60.0 dB
(def tl-spread (tl/spherical-spreading-loss 1000.0))
(h/assert-near 60.0 tl-spread 0.01 "Spherical spreading loss at 1000m is exactly 60 dB")

# Total transmission loss at 1000m and 25 kHz
(def tl-total (tl/total-transmission-loss 1000.0 25.0))
(h/assert-near (+ 60.0 alpha-25) tl-total 0.01 "Total TL equals spreading plus 1km absorption")

# Wenz ambient noise at 10 kHz with 15 knots wind
(def nl-wenz (tl/wenz-ambient-noise-level 10000.0 15.0 0.5))
(h/assert-true (> nl-wenz 40.0) "Ambient noise at 10 kHz is above 40 dB")
(h/assert-true (< nl-wenz 75.0) "Ambient noise at 10 kHz is below 75 dB")

# Passive sonar equation: SL=180 dB, range=2000m, freq=12kHz, NL=60dB, DI=15dB
(def snr-pass (tl/sonar-snr-passive 180.0 2000.0 12.0 60.0 15.0))
(h/assert-true (> snr-pass 10.0) "Passive sonar detection SNR is positive and above 10 dB")

# -----------------------------------------------------------------------------
# USBL Positioning Tests
# -----------------------------------------------------------------------------

(def usbl-sys (usbl/make-usbl-transceiver 0.10 25000.0 1500.0 [0.0 0.0 2.0]))
(def dir-cos (usbl/phase-diff-to-direction-cosines usbl-sys 0.0 0.0))
(h/assert-vec-near [0.0 0.0 1.0] dir-cos 1e-6 "Zero phase difference corresponds to nadir direction [0, 0, 1]")

# Slant range from 1.0 s TWTT at c=1500 m/s: R = 1500 * 1.0 / 2 = 750 m
(def r-slant (usbl/twtt-to-slant-range 1.0 1500.0))
(h/assert-near 750.0 r-slant 1e-6 "Slant range for 1.0s TWTT is 750 meters")

# Vehicle positioning test with ship at origin and identity attitude
(def ship-pos [0.0 0.0 0.0])
(def ship-att m/mat3-identity)
(def auv-pos-ned (usbl/resolve-usbl-position-ned usbl-sys ship-pos ship-att 0.0 0.0 1.0 1500.0))
# Transducer lever arm is 2.0m down, plus 750m down = 752m down
(h/assert-vec-near [0.0 0.0 752.0] auv-pos-ned 1e-3 "AUV positioned directly below ship at 752m depth")

# Covariance test
(def cov-usbl (usbl/usbl-covariance-ned 750.0 ship-att 0.02 0.20))
(h/assert-true (> (in cov-usbl 0) 100.0) "Cross-range variance scales with range squared")
(h/assert-near 0.04 (in cov-usbl 8) 1e-3 "Depth/range variance matches range noise squared (0.2^2 = 0.04)")

# -----------------------------------------------------------------------------
# LBL Multilateration Tests
# -----------------------------------------------------------------------------

# Define a 4-beacon seabed array around a 1000m x 1000m survey site at 1500m depth
(def b1 (lbl/make-transponder "B1" [-500.0 -500.0 1500.0] 0.02))
(def b2 (lbl/make-transponder "B2" [ 500.0 -500.0 1500.0] 0.02))
(def b3 (lbl/make-transponder "B3" [ 500.0  500.0 1500.0] 0.02))
(def b4 (lbl/make-transponder "B4" [-500.0  500.0 1500.0] 0.02))
(def lbl-array [b1 b2 b3 b4])

# True AUV position: [100.0, 50.0, 1400.0] (100m above seabed)
(def true-auv [100.0 50.0 1400.0])

# Compute synthetic one-way ranges to each beacon
(def ranges @[])
(each b lbl-array
  (def d (m/vec3-dist true-auv (get b :pos)))
  (array/push ranges d))

# Initial guess perturbed by 50 meters
(def init-guess [150.0 80.0 1430.0])
(def lbl-sol (lbl/solve-lbl-position lbl-array ranges init-guess))

(h/assert-true (get lbl-sol :converged) "LBL Gauss-Newton multilateration converged")
(def solved-pos (get lbl-sol :pos))
(h/assert-vec-near true-auv solved-pos 1e-3 "LBL solved position matches true AUV position within 1 mm")

# DOP calculation
(def dop (lbl/compute-lbl-dop (get lbl-sol :cov)))
(h/assert-true (< (get dop :hdop) 3.0) "HDOP is within favorable geometric limits (< 3.0)")
(h/assert-true (< (get dop :gdop) 5.0) "GDOP is within favorable geometric limits (< 5.0)")

(h/summarize-tests "Underwater Acoustics, USBL, and LBL Positioning")
