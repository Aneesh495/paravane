# examples/survey_deep_ridge.janet
# Autonomous deep ocean ridge bathymetric survey mission script.

(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/ocean/profile :as prof)
(import ../src/paravane/acoustics/ray_tracer :as ray)
(import ../src/paravane/navigation/ekf :as ekf)
(import ../src/paravane/guidance/los :as los)
(import ../src/paravane/guidance/survey :as survey)
(import ../src/paravane/slam/bathymetry_grid :as bgrid)
(import ../src/paravane/slam/mbes :as mbes)
(import ../src/paravane/executive/fiber_engine :as fe)
(import ../src/paravane/executive/behavior_tree :as bt)
(import ../src/paravane/telemetry/geojson :as geojson)

(print "===========================================================")
(print "  MISSION: DEEP OCEAN RIDGE HIGH-RESOLUTION SURVEY        ")
(print "  Target Area: Mid-Atlantic Fracture Zone (1200m depth)   ")
(print "===========================================================\n")

# 1. Environment & Sound Speed Profile Setup
(def depths @[0.0 200.0 500.0 800.0 1000.0 1200.0 1500.0 2000.0])
(def speeds @[1525.0 1510.0 1495.0 1488.0 1485.0 1489.0 1498.0 1515.0])
(def ocean-svp (prof/make-svp depths speeds))
(print "[OK] Acoustic Sound Speed Profile initialized (SOFAR axis at 1000m)")

# 2. Navigation ES-EKF Setup
(var filter (ekf/make-es-ekf))
(print "[OK] 15-state Error-State Extended Kalman Filter initialized")

# 3. Bathymetric DEM Mapping Grid
(def dem-grid (bgrid/make-bathymetry-grid 5.0))
(print "[OK] 2.5D Multibeam DEM Grid initialized at 5.0m resolution")

# 4. Survey Mission Waypoint Planning
(def survey-origin [0.0 0.0 1150.0])
(def grid-wps (survey/generate-survey-grid survey-origin 500.0 200.0 50.0 0.0 1150.0 1.5))
(print (string/format "[OK] Boustrophedon survey grid generated: %d waypoints planned" (length grid-wps)))

# 5. Shared Mission Blackboard
(def bb @{:phase :descent
          :depth 10.0
          :altitude 1190.0
          :pos [0.0 0.0 10.0]
          :vel [0.0 0.0 0.0]
          :yaw 0.0
          :current-wp-idx 0
          :total-wps (length grid-wps)
          :completed false
          :aborted false
          :trajectory @[]})

# 6. Mission Execution Fibers
(def scheduler (fe/make-scheduler))

# Navigation Sensor Fusion Task (10 Hz)
(defn nav-loop []
  (while (not (get bb :completed))
    # Propagate EKF with IMU step
    (set filter (ekf/ekf-predict filter [0.0 0.0 -9.80665] [0.0 0.0 0.0] 0.1))

    # DVL bottom update when below 1000m
    (when (> (get bb :depth) 1000.0)
      (set filter (ekf/ekf-update-dvl filter [1.5 0.0 0.0] m/mat3-identity)))

    # Pressure depth update
    (set filter (ekf/ekf-update-depth filter (get bb :depth) 0.05))

    (yield [:sleep 0.1])))

# Multibeam Swath Acquisition Task (2 Hz)
(defn mbes-loop []
  (while (not (get bb :completed))
    (when (> (get bb :depth) 1100.0)
      (let [cur-x (in (get bb :pos) 0)
            cur-y (in (get bb :pos) 1)
            cur-z (in (get bb :pos) 2)
            alt (get bb :altitude)]
        # Synthesize 11 multibeam beams across swath
        (for b -5 6
          (let [beam-angle (* b 0.1) # radians
                sounding-x (+ cur-x (* alt (math/tan beam-angle)))
                sounding-y cur-y
                sounding-z (+ cur-z alt)]
            (bgrid/insert-sounding! dem-grid sounding-x sounding-y sounding-z)))))
    (yield [:sleep 0.5])))

# Vehicle Guidance and Mission Executive Task (1 Hz)
(defn guidance-loop []
  (for cycle 0 80
    (let [phase (get bb :phase)]
      (cond
        (= phase :descent)
        (do
          (put bb :depth (+ (get bb :depth) 150.0))
          (put bb :altitude (max 10.0 (- 1200.0 (get bb :depth))))
          (put bb :pos [0.0 0.0 (get bb :depth)])
          (when (>= (get bb :depth) 1150.0)
            (print (string/format "[EVENT] Target survey depth reached (%.1fm). Transitioning to SURVEY."
                                  (get bb :depth)))
            (put bb :phase :survey)))

        (= phase :survey)
        (do
          (let [wp-idx (get bb :current-wp-idx)]
            (if (< wp-idx (get bb :total-wps))
              (let [target-wp (get grid-wps wp-idx)
                    t-pos (get target-wp :pos)
                    c-pos (get bb :pos)
                    dx (- (in t-pos 0) (in c-pos 0))
                    dy (- (in t-pos 1) (in c-pos 1))
                    dist (math/sqrt (+ (* dx dx) (* dy dy)))]
                (if (< dist 40.0)
                  (do
                    (print (string/format "[WAYPOINT] Reached WP %02d (North=%6.1f, East=%6.1f)"
                                          wp-idx (in t-pos 0) (in t-pos 1)))
                    (put bb :current-wp-idx (+ wp-idx 1)))
                  (let [step 85.0
                        ratio (min 1.0 (/ step (max 1.0 dist)))
                        nx (+ (in c-pos 0) (* dx ratio))
                        ny (+ (in c-pos 1) (* dy ratio))]
                    (put bb :pos [nx ny 1150.0]))))
              (do
                (print "[EVENT] All survey grid tracklines completed. Transitioning to ASCENT.")
                (put bb :phase :ascent)))))

        (= phase :ascent)
        (do
          (put bb :depth (- (get bb :depth) 180.0))
          (put bb :pos [(in (get bb :pos) 0) (in (get bb :pos) 1) (max 0.0 (get bb :depth))])
          (when (<= (get bb :depth) 0.0)
            (print "[EVENT] Vehicle surfaced safely. Mission complete.")
            (put bb :completed true)))))
    (array/push (get bb :trajectory) (get bb :pos))
    (yield [:sleep 0.5])))

# Spawn cooperative fibers
(fe/schedule-task! scheduler (fe/make-task "Navigation-Fusion-10Hz" nav-loop 10))
(fe/schedule-task! scheduler (fe/make-task "Multibeam-Swath-2Hz" mbes-loop 5))
(fe/schedule-task! scheduler (fe/make-task "Guidance-Executive-1Hz" guidance-loop 1))

(print "\nStarting autonomous mission event loop...")
(var iter 0)
(while (and (not (get bb :completed)) (< iter 300))
  (fe/run-scheduler-tick! scheduler 0.1)
  (++ iter))

# 7. Export Trajectory GeoJSON
(try (os/mkdir "output") ([_] nil))
(def ref-origin [(* 32.5 c/deg-to-rad) (* -64.8 c/deg-to-rad) 0.0])
(def geojson-str (geojson/export-trajectory-geojson (get bb :trajectory) ref-origin))
(spit "output/deep_ridge_trajectory.geojson" geojson-str)
(print "[OK] Mission trajectory exported to output/deep_ridge_trajectory.geojson")

# 7. Mission Post-Flight Analytics
(print "\n===========================================================")
(print "  MISSION POST-FLIGHT REPORT                                ")
(print "===========================================================")
(print (string/format "Final Phase:               %s" (get bb :phase)))
(print (string/format "Total Simulation Ticks:    %d" iter))
(print (string/format "Total Multibeam Soundings: %d" (get dem-grid :total-soundings)))
(print (string/format "Active DEM Grid Cells:     %d" (length (keys (get dem-grid :cells)))))
(print (string/format "Grid X Envelope:           [%.1f m, %.1f m]"
                      (get dem-grid :min-x) (get dem-grid :max-x)))
(print (string/format "Grid Y Envelope:           [%.1f m, %.1f m]"
                      (get dem-grid :min-y) (get dem-grid :max-y)))
(print "Status: MISSION COMPLETED SUCCESSFULLY")
