# test/test_e2e_mission.janet
# Comprehensive end-to-end integration test suite for autonomous AUV missions.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/ocean/profile :as prof)
(import ../src/paravane/acoustics/ray_tracer :as ray)
(import ../src/paravane/navigation/ekf :as ekf)
(import ../src/paravane/guidance/los :as los)
(import ../src/paravane/guidance/survey :as survey)
(import ../src/paravane/slam/bathymetry_grid :as bgrid)
(import ../src/paravane/executive/fiber_engine :as fe)
(import ../src/paravane/executive/behavior_tree :as bt)
(import ../src/paravane/executive/fault_manager :as fdir)
(import ../src/paravane/telemetry/geojson :as geojson)

# -----------------------------------------------------------------------------
# 1. Closed-Loop Mission Execution Verification
# -----------------------------------------------------------------------------

(defn test-closed-loop-mission []
  (def scheduler (fe/make-scheduler))
  (def dem-grid (bgrid/make-bathymetry-grid 5.0))
  (var nav-filter (ekf/make-es-ekf))

  (def wps (survey/generate-survey-grid [0.0 0.0 1000.0] 200.0 100.0 50.0 0.0 1000.0 1.5))
  (h/assert-equal 6 (length wps) "Grid must generate 6 survey waypoints for 200x100m at 50m spacing")

  (def bb @{:phase :descent
            :depth 10.0
            :pos [0.0 0.0 10.0]
            :wp-idx 0
            :completed false})

  (defn mission-loop []
    (for step 0 60
      (let [phase (get bb :phase)]
        (cond
          (= phase :descent)
          (do
            (put bb :depth (+ (get bb :depth) 250.0))
            (put bb :pos [0.0 0.0 (get bb :depth)])
            (when (>= (get bb :depth) 1000.0)
              (put bb :phase :survey)))

          (= phase :survey)
          (do
            (let [cur-wp-idx (get bb :wp-idx)]
              (if (< cur-wp-idx (length wps))
                (let [tgt (get (get wps cur-wp-idx) :pos)
                      cur (get bb :pos)
                      dx (- (in tgt 0) (in cur 0))
                      dy (- (in tgt 1) (in cur 1))
                      dist (math/sqrt (+ (* dx dx) (* dy dy)))]
                  (if (< dist 30.0)
                    (put bb :wp-idx (+ cur-wp-idx 1))
                    (let [ratio (min 1.0 (/ 75.0 (max 1.0 dist)))]
                      (put bb :pos [(+ (in cur 0) (* dx ratio))
                                    (+ (in cur 1) (* dy ratio))
                                    1000.0]))))
                (put bb :phase :ascent))))

          (= phase :ascent)
          (do
            (put bb :depth (- (get bb :depth) 300.0))
            (put bb :pos [(in (get bb :pos) 0) (in (get bb :pos) 1) (max 0.0 (get bb :depth))])
            (when (<= (get bb :depth) 0.0)
              (put bb :completed true)))))

      # Insert soundings during survey
      (when (= (get bb :phase) :survey)
        (bgrid/insert-sounding! dem-grid (in (get bb :pos) 0) (in (get bb :pos) 1) 1030.0))

      (yield [:sleep 0.1])))

  (fe/schedule-task! scheduler (fe/make-task "Mission" mission-loop 1))

  (for i 0 100
    (fe/run-scheduler-tick! scheduler 0.1))

  (h/assert-true (get bb :completed) "Mission must successfully surface and complete")
  (h/assert-true (> (get dem-grid :total-soundings) 5) "Multibeam soundings must be collected"))

# -----------------------------------------------------------------------------
# 2. ES-EKF Filter Covariance Convergence Test
# -----------------------------------------------------------------------------

(defn test-ekf-covariance-convergence []
  (var filter (ekf/make-es-ekf))

  # Propagate 20 seconds of dead reckoning: covariance grows
  (for i 0 20
    (set filter (ekf/ekf-predict filter [0.0 0.0 -9.80665] [0.0 0.0 0.0] 1.0)))

  (def p-cov-dr (get filter :cov))
  (def var-z-dr (ekf/mat15-get p-cov-dr 2 2)) # Depth variance

  # Fuse high-precision depth sensor measurement: covariance shrinks
  (set filter (ekf/ekf-update-depth filter 500.0 0.0025))
  (def p-cov-fused (get filter :cov))
  (def var-z-fused (ekf/mat15-get p-cov-fused 2 2))

  (h/assert-true (< var-z-fused var-z-dr) "Depth update must strictly reduce depth error covariance")
  (h/assert-true (< var-z-fused 0.05) "Post-update depth variance must be bounded by sensor noise"))

# -----------------------------------------------------------------------------
# 3. FDIR Geofence Breach Override Test
# -----------------------------------------------------------------------------

(defn test-fdir-geofencing []
  # Define survey operations boundary polygon
  (def poly-boundary @[[0.0 0.0] [500.0 0.0] [500.0 300.0] [0.0 300.0]])

  (def inside-pos [250.0 150.0 1000.0])
  (def breach-pos [750.0 150.0 1000.0]) # Outside eastern boundary

  (h/assert-true (fdir/is-point-inside-polygon-2d [(in inside-pos 0) (in inside-pos 1)] poly-boundary)
                 "Vehicle inside operations box must be inside geofence")

  (h/assert-false (fdir/is-point-inside-polygon-2d [(in breach-pos 0) (in breach-pos 1)] poly-boundary)
                  "Vehicle outside operations box must trigger geofence breach"))

# -----------------------------------------------------------------------------
# 4. Behavior Tree Mission Executive Switching
# -----------------------------------------------------------------------------

(defn test-bt-mission-executive []
  (def bb @{:battery 85.0 :water-ingress false :emergency false})

  (def abort-check
    (bt/bt-condition "Abort-Check" (fn [b] (or (< (get b :battery) 15.0) (get b :water-ingress)))))

  (h/assert-equal :failure (bt/bt-tick abort-check bb 0.1) "Safe vehicle state must report failure to abort check")

  # Simulate hull compromise
  (put bb :water-ingress true)
  (h/assert-equal :success (bt/bt-tick abort-check bb 0.1) "Water ingress must trigger abort condition"))

# -----------------------------------------------------------------------------
# Execute All E2E Integration Tests
# -----------------------------------------------------------------------------

(defn run-tests []
  (h/reset-test-stats)
  (test-closed-loop-mission)
  (test-ekf-covariance-convergence)
  (test-fdir-geofencing)
  (test-bt-mission-executive)
  (h/summarize-tests "End-to-End AUV Mission Scenarios"))

(run-tests)
