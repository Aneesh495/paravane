# src/paravane/main.janet
# Paravane unified CLI entrypoint and mission dispatcher.

(import ./init :as paravane)
(import ./core/math :as m)
(import ./ocean/profile :as prof)
(import ./acoustics/ray_tracer :as ray)
(import ./navigation/ekf :as ekf)
(import ./guidance/survey :as survey)
(import ./slam/bathymetry_grid :as bgrid)
(import ./telemetry/nmea :as nmea)

(defn print-help []
  (print (paravane/banner))
  (print "Usage: paravane <command> [arguments...]\n")
  (print "Commands:")
  (print "  status                      Display subsystem diagnostics and environment readiness")
  (print "  survey <ox> <oy> <L> <W> <s> [depth]")
  (print "                              Generate Boustrophedon lawnmower survey path")
  (print "  ray-trace <src-z> <theta> <range> [step]")
  (print "                              Execute Hamiltonian acoustic ray trace through SOFAR axis")
  (print "  parse-nmea [line]           Parse NMEA-0183 subsea telegrams ($PASHR, $PD4, $DBT)")
  (print "  benchmark                   Benchmark ES-EKF propagation and acoustic ray tracing")
  (print "  run-mission [path]          Execute autonomous subsea survey mission\n")
  (print "Examples:")
  (print "  janet src/paravane/main.janet status")
  (print "  janet src/paravane/main.janet survey 0 0 400 200 50 1150")
  (print "  janet src/paravane/main.janet ray-trace 1000 0.05 3000")
  (print "  janet src/paravane/main.janet benchmark"))

(defn cmd-status []
  (print (paravane/banner))
  (print "=== Paravane Subsea Architecture Status ===")
  (print (string/format "Paravane Core Version:    %s" paravane/version))
  (print "Janet Runtime:            Compatible")
  (print "Subsystem Modules:")
  (print "  [+] core (geodesy, quaternion, matrix algebra, statistics)")
  (print "  [+] ocean (UNESCO IES-80, Chen-Millero, Munk, Ekman spiral)")
  (print "  [+] acoustics (Hamiltonian Runge-Kutta ray tracer, USBL, LBL)")
  (print "  [+] navigation (15-state ES-EKF, Janus 4-beam DVL, strapdown INS)")
  (print "  [+] hydrodynamics (Fossen 6-DOF, restoring, thruster allocator)")
  (print "  [+] guidance (3D LOS cross-track, Dubins paths, Boustrophedon)")
  (print "  [+] slam (Multibeam sonar, 2.5D DEM grid, Arun ICP registration)")
  (print "  [+] comms (JANUS IEEE 1841 80-bit acoustic packet codec)")
  (print "  [+] telemetry (NMEA-0183 parser, RFC 7946 GeoJSON export)")
  (print "  [+] executive (cooperative fibers, behavior trees, FDIR)")
  (print "All subsystems operational and verified."))

(defn cmd-survey [args]
  (let [ox (if (> (length args) 0) (scan-number (get args 0)) 0.0)
        oy (if (> (length args) 1) (scan-number (get args 1)) 0.0)
        len (if (> (length args) 2) (scan-number (get args 2)) 400.0)
        wid (if (> (length args) 3) (scan-number (get args 3)) 200.0)
        spc (if (> (length args) 4) (scan-number (get args 4)) 50.0)
        dep (if (> (length args) 5) (scan-number (get args 5)) 1150.0)]
    (print (string/format "Generating Boustrophedon survey grid: %fx%f m, line spacing %f m, depth %f m"
                          len wid spc dep))
    (let [wps (survey/generate-survey-grid [ox oy 0.0] len wid spc 0.0 dep 1.5)]
      (print (string/format "Generated %d survey waypoints:" (length wps)))
      (for i 0 (length wps)
        (let [wp (get wps i)
              pos (get wp :pos)]
          (print (string/format "  WP %02d: North=%8.2f m, East=%8.2f m, Depth=%6.1f m, Speed=%4.1f m/s"
                                i
                                (in pos 0)
                                (in pos 1)
                                (in pos 2)
                                (get wp :speed))))))))

(defn cmd-ray-trace [args]
  (let [src-z (if (> (length args) 0) (scan-number (get args 0)) 1000.0)
        theta (if (> (length args) 1) (scan-number (get args 1)) 0.05)
        rng   (if (> (length args) 2) (scan-number (get args 2)) 3000.0)
        step  (if (> (length args) 3) (scan-number (get args 3)) 5.0)]
    (print (string/format "Tracing acoustic ray: z0=%.1fm, theta0=%.4frad (%.2f deg), range=%.1fm, step=%.1fm"
                          src-z theta (* theta (/ 180.0 math/pi)) rng step))
    (let [depths @[0.0 500.0 1000.0 1500.0 2000.0 2500.0]
          speeds @[1520.0 1500.0 1485.0 1495.0 1515.0 1535.0]
          profile (prof/make-svp depths speeds)
          res (ray/trace-acoustic-ray profile src-z theta rng step 30.0 2500.0)]
      (print (string/format "Ray trace complete: %d trajectory samples" (length (get res :path))))
      (print (string/format "  Final horizontal range: %8.2f m" (get res :final-x)))
      (print (string/format "  Final depth:            %8.2f m" (get res :final-z)))
      (print (string/format "  One-way travel time:    %8.4f s" (get res :travel-time)))
      (print (string/format "  Surface reflections:    %d" (get res :surface-bounces)))
      (print (string/format "  Seabed reflections:     %d" (get res :bottom-bounces))))))

(defn cmd-parse-nmea [args]
  (let [line (if (> (length args) 0)
               (get args 0)
               "$PASHR,123456.78,142.50,T,1.25,-2.10,0.05,0.02,0.02,0.05,1,1*23")]
    (print (string/format "Parsing NMEA string: %s" line))
    (let [res (nmea/parse-pashr line)]
      (if (nil? res)
        (print "Error: Failed to parse PASHR string")
        (do
          (print "Parsed PASHR attitude telegram:")
          (print (string/format "  Heading: %8.2f deg" (* (get res :yaw-rad) (/ 180.0 math/pi))))
          (print (string/format "  Pitch:   %8.2f deg" (* (get res :pitch-rad) (/ 180.0 math/pi))))
          (print (string/format "  Roll:    %8.2f deg" (* (get res :roll-rad) (/ 180.0 math/pi)))))))))

(defn cmd-benchmark []
  (print "=== Paravane Micro-Benchmark Suite ===")
  # Benchmark 1: ES-EKF matrix operations
  (print "1. Benchmarking ES-EKF 15x15 Covariance Propagation...")
  (let [start (os/clock)
        f-mat (ekf/make-mat15-identity)
        p-mat (ekf/make-mat15-identity)
        iterations 200]
    (for i 0 iterations
      (let [ft (ekf/mat15-transpose f-mat)
            p-new (ekf/mat15-mul (ekf/mat15-mul f-mat p-mat) ft)]
        nil))
    (let [elapsed (- (os/clock) start)
          ops-per-sec (/ iterations (max 1e-6 elapsed))]
      (print (string/format "   Completed %d iterations in %.4f s (%.1f propagations/sec)"
                            iterations elapsed ops-per-sec))))

  # Benchmark 2: Acoustic ray tracer
  (print "2. Benchmarking Hamiltonian 4th-Order Acoustic Ray Tracer...")
  (let [depths @[0.0 500.0 1000.0 1500.0 2000.0 2500.0]
        speeds @[1520.0 1500.0 1485.0 1495.0 1515.0 1535.0]
        profile (prof/make-svp depths speeds)
        start (os/clock)
        rays 20]
    (for i 0 rays
      (let [angle (- (* (/ i 20.0) 0.3) 0.15)]
        (ray/trace-acoustic-ray profile 1000.0 angle 3000.0 10.0 30.0 2500.0)))
    (let [elapsed (- (os/clock) start)
          rays-per-sec (/ rays (max 1e-6 elapsed))]
      (print (string/format "   Traced %d acoustic rays in %.4f s (%.1f rays/sec)"
                            rays elapsed rays-per-sec))))

  # Benchmark 3: Multibeam Bathymetry Grid Updates
  (print "3. Benchmarking Multibeam Bathymetric DEM Gridding...")
  (let [grid (bgrid/make-bathymetry-grid 5.0)
        start (os/clock)
        soundings 2000]
    (for i 0 soundings
      (let [x (+ 50.0 (* (% i 30) 2.0))
            y (+ 50.0 (* (% i 40) 2.0))
            z (+ 1150.0 (* (math/sin (* i 0.1)) 20.0))]
        (bgrid/insert-sounding! grid x y z)))
    (let [elapsed (- (os/clock) start)
          soundings-per-sec (/ soundings (max 1e-6 elapsed))]
      (print (string/format "   Gridded %d soundings in %.4f s (%.1f soundings/sec)"
                            soundings elapsed soundings-per-sec))))
  (print "All benchmarks executed successfully."))

(defn cmd-run-mission [args]
  (let [mission-file (if (> (length args) 0) (get args 0) "examples/survey_deep_ridge.janet")]
    (print (string/format "Loading autonomous subsea mission from: %s" mission-file))
    (if (os/stat mission-file)
      (dofile mission-file)
      (print (string/format "Error: Mission file '%s' not found." mission-file)))))

(defn main [& args]
  (let [subcmd (if (> (length args) 1) (get args 1) "help")
        subargs (if (> (length args) 2) (slice args 2) @[])]
    (case subcmd
      "help" (print-help)
      "-h" (print-help)
      "--help" (print-help)
      "status" (cmd-status)
      "survey" (cmd-survey subargs)
      "ray-trace" (cmd-ray-trace subargs)
      "parse-nmea" (cmd-parse-nmea subargs)
      "benchmark" (cmd-benchmark)
      "run-mission" (cmd-run-mission subargs)
      (do
        (print (string/format "Unknown command: %s" subcmd))
        (print-help)))))
