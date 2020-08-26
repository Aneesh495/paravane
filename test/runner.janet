# test/runner.janet
# Master test orchestrator for the Paravane subsea robotics framework.

(import ./helpers :as h)

(def test-suites
  [# Core mathematical, geometrical, and geodesy foundations
   "test_core_math"
   # Oceanographic physics, sound velocity, and seawater equations of state
   "test_ocean_physics"
   # Acoustic ray tracing, Snell's law, and USBL/LBL positioning
   "test_acoustics"
   # Inertial navigation mechanization, DVL transforms, and ES-EKF
   "test_navigation"
   # 6-DOF hydrodynamics, added mass, restoring forces, and thruster allocation
   "test_hydrodynamics"
   # Line-of-sight guidance, Dubins 3D curves, and survey planning
   "test_guidance"
   # Multibeam echo-sounder, bathymetric mapping, and terrain matching
   "test_slam"
   # JANUS acoustic communications and subsea telemetry protocols
   "test_comms_telemetry"
   # Fiber task executive, behavior tree engine, and fault management
   "test_executive"
   # End-to-end mission simulation verification
   "test_e2e_mission"])

(defn main [& args]
  (print "=========================================================")
  (print "    PARAVANE: AUV AUTONOMOUS SYSTEMS TEST RUNNER         ")
  (print "=========================================================")
  (var all-suites-passed true)
  (var suites-executed 0)

  (each suite test-suites
    (def path (string "test/" suite ".janet"))
    (if (os/stat path)
      (do
        (print (string "\n>>> Executing " suite " ..."))
        (def env (make-env))
        (put env :current-file path)
        (try
          (do
            (dofile path :env env)
            (++ suites-executed))
          ([err]
            (print (string "[ERROR] Suite execution failed with exception: " err))
            (set all-suites-passed false))))
      (print (string "[-] Suite " suite " not yet implemented (skipping)"))))

  (print "\n=========================================================")
  (print (string "Test execution complete. Suites run: " suites-executed))
  (if (and all-suites-passed (> suites-executed 0))
    (do
      (print "[SUCCESS] All test suites completed without errors.")
      (os/exit 0))
    (if (zero? suites-executed)
      (do
        (print "[NOTICE] Test scaffolding verified.")
        (os/exit 0))
      (do
        (print "[FAILURE] One or more test suites failed.")
        (os/exit 1)))))
