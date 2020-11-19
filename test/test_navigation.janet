# test/test_navigation.janet
# Comprehensive unit tests for IMU models, DVL transforms, INS mechanization, and 15-state ES-EKF.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/navigation/sensors :as sens)
(import ../src/paravane/navigation/dvl :as dvl)
(import ../src/paravane/navigation/strapdown :as ins)
(import ../src/paravane/navigation/ekf :as ekf)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# DVL Beam Transform Tests
# -----------------------------------------------------------------------------

(def dvl-inst (dvl/make-janus-dvl 30.0 1500.0))

# Vehicle surging forward at 1.5 m/s, zero sway, zero heave
(def true-v-surge [1.5 0.0 0.0])
(def beam-vels (dvl/body-to-beam-velocities dvl-inst true-v-surge))

# In 30 deg Janus: beam 1 (fwd) = 1.5 * sin(30) = 0.75 m/s
# beam 3 (aft) = -1.5 * sin(30) = -0.75 m/s
# beam 2 (stbd) and 4 (port) = 0.0 m/s
(h/assert-near 0.75 (in beam-vels 0) 1e-6 "Beam 1 measures forward component * sin(30)")
(h/assert-near 0.0 (in beam-vels 1) 1e-6 "Beam 2 measures zero for purely forward motion")
(h/assert-near -0.75 (in beam-vels 2) 1e-6 "Beam 3 measures negative aft component")
(h/assert-near 0.0 (in beam-vels 3) 1e-6 "Beam 4 measures zero for purely forward motion")

# Reconstruct body velocity via 4-beam solver
(def recon-4beam (dvl/beam-to-body-velocities dvl-inst beam-vels))
(h/assert-vec-near true-v-surge (get recon-4beam :vel) 1e-6 "4-beam inverse reconstruction recovers true surge velocity")
(h/assert-near 0.0 (get recon-4beam :error-velocity) 1e-6 "Error velocity is zero for consistent synthetic beams")

# Test 3-beam fallback when Beam 1 (forward) drops lock
(def recon-3beam (dvl/beam-to-body-3beam-fallback dvl-inst beam-vels 0))
(h/assert-vec-near true-v-surge (get recon-3beam :vel) 1e-6 "3-beam fallback recovers surge velocity with Beam 1 dropped")

# Test sound speed scaling correction: 1530 m/s actual vs 1500 m/s calibration
(def v-corr (dvl/correct-dvl-sound-speed [1.5 0.0 0.0] 1530.0 1500.0))
(h/assert-near 1.53 (in v-corr 0) 1e-6 "Sound speed scaling scales velocity proportionally (1.5 * 1.02 = 1.53)")

# -----------------------------------------------------------------------------
# Strapdown Inertial Navigation (INS) Tests
# -----------------------------------------------------------------------------

# Static vehicle resting at 100m depth, level attitude
# Accelerometer measures specific force counteracting gravity: [0, 0, -g]
(def g-local (c/gravity-at-depth (* 45.0 c/deg-to-rad) 100.0))
(def static-accel [0.0 0.0 (- g-local)])
(def static-omega [0.0 0.0 0.0])

(def init-ins (ins/make-ins-state [0.0 0.0 100.0] [0.0 0.0 0.0] m/quat-identity))
(var ins-sim init-ins)

# Propagate for 100 steps of dt = 0.01s (1.0 second total)
(for i 0 100
  (set ins-sim (ins/ins-mechanize-step ins-sim static-accel static-omega 0.01)))

(h/assert-vec-near [0.0 0.0 0.0] (get ins-sim :vel) 1e-3 "Static INS maintains zero velocity under gravity balance")
(h/assert-near 100.0 (in (get ins-sim :pos) 2) 1e-3 "Static INS maintains constant 100m depth")

# -----------------------------------------------------------------------------
# 15-State Error-State Extended Kalman Filter (ES-EKF) Tests
# -----------------------------------------------------------------------------

(def filter (ekf/make-es-ekf [0.0 0.0 50.0] [1.0 0.0 0.0] m/quat-identity))

# Prediction step: simulate 0.5s forward propagation
(var cur-filt filter)
(for i 0 50
  (set cur-filt (ekf/ekf-predict cur-filt static-accel static-omega 0.01)))

(h/assert-near 0.5 (get cur-filt :time) 1e-6 "Filter time tracks elapsed simulation time")
(def pos-after-pred (get (get cur-filt :ins) :pos))
# 1.0 m/s surge * 0.5s = 0.5m North
(h/assert-near 0.5 (in pos-after-pred 0) 0.05 "Forward propagation advances position correctly")

# Depth transducer update: depth sensor reports true depth 50.02m
(def cur-filt-depth (ekf/ekf-update-depth cur-filt 50.02 0.0025))
(def depth-after (in (get (get cur-filt-depth :ins) :pos) 2))
(h/assert-near 50.01 depth-after 0.02 "Depth measurement update corrects vertical position")

# DVL velocity update: DVL reports [0.98, 0.01, 0.00] m/s body velocity
(def r-dvl (m/mat3-diag 0.0001 0.0001 0.0001)) # (0.01 m/s)^2 variance
(def cur-filt-dvl (ekf/ekf-update-dvl cur-filt-depth [0.98 0.01 0.0] r-dvl))
(def vel-after-dvl (get (get cur-filt-dvl :ins) :vel))
(h/assert-near 0.98 (in vel-after-dvl 0) 0.02 "DVL measurement update corrects forward velocity")

# Acoustic position fix update (USBL fix: [0.55, 0.02, 50.01])
(def r-usbl (m/mat3-diag 0.25 0.25 0.04))
(def cur-filt-usbl (ekf/ekf-update-acoustic-pos cur-filt-dvl [0.55 0.02 50.01] r-usbl))
(def pos-after-usbl (get (get cur-filt-usbl :ins) :pos))
(h/assert-near 0.55 (in pos-after-usbl 0) 0.10 "Acoustic position update bounds horizontal drift")

(h/summarize-tests "Inertial Navigation, DVL Processing, and ES-EKF Fusion")
