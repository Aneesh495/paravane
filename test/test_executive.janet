# test/test_executive.janet
# Comprehensive unit tests for fiber task scheduling, behavior tree engine, and FDIR safety executive.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/executive/fiber_engine :as fe)
(import ../src/paravane/executive/behavior_tree :as bt)
(import ../src/paravane/executive/fault_manager :as fdir)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# Fiber Task Scheduler Tests
# -----------------------------------------------------------------------------

(def sched (fe/make-scheduler))
(var counter-a 0)
(var counter-b 0)

(def task-a (fe/make-task "CounterA" (fn [dt]
                                       (for i 0 5
                                         (++ counter-a)
                                         (yield :idle))) 1))

(def task-b (fe/make-task "CounterB" (fn [dt]
                                       (for i 0 3
                                         (++ counter-b)
                                         (yield [:sleep 0.20]))) 2))

(fe/schedule-task! sched task-a)
(fe/schedule-task! sched task-b)

# Run 10 ticks of 0.05 seconds
(for i 0 10
  (fe/run-scheduler-tick! sched 0.05))

(h/assert-true (>= counter-a 5) "Task A completes all 5 loop iterations")
(h/assert-true (> counter-b 0) "Task B executes with sleep intervals")

# -----------------------------------------------------------------------------
# Behavior Tree Engine Tests
# -----------------------------------------------------------------------------

(def bb @{:battery 0.80
          :depth 100.0
          :mission-step 0})

# Condition: battery > 20%
(def cond-battery (bt/bt-condition "CheckBattery" (fn [b] (> (get b :battery) 0.20))))
(h/assert-equal :success (bt/bt-tick cond-battery bb 0.1) "Battery condition passes for 80% charge")

# Action: increment step
(def act-step (bt/bt-action "IncStep" (fn [b dt]
                                        (put b :mission-step (+ (get b :mission-step) 1))
                                        :success)))

# Sequence: CheckBattery -> IncStep
(def seq-node (bt/bt-sequence "SurveySeq" [cond-battery act-step]))
(def seq-status (bt/bt-tick seq-node bb 0.1))

(h/assert-equal :success seq-status "Sequence succeeds when all children succeed")
(h/assert-equal 1 (get bb :mission-step) "Action inside sequence updated blackboard state")

# Selector test with failing first branch
(def cond-fail (bt/bt-condition "AlwaysFail" (fn [b] false)))
(def sel-node (bt/bt-selector "FallbackSel" [cond-fail cond-battery]))
(h/assert-equal :success (bt/bt-tick sel-node bb 0.1) "Selector falls back to succeeding second branch")

# Inverter test
(def inv-node (bt/bt-inverter "InvertFail" cond-fail))
(h/assert-equal :success (bt/bt-tick inv-node bb 0.1) "Inverter turns failure into success")

# -----------------------------------------------------------------------------
# FDIR Safety and Geofencing Tests
# -----------------------------------------------------------------------------

# Define rectangular safe survey operational polygon: [0, 0] to [1000, 500]
(def geofence @[[0.0 0.0] [1000.0 0.0] [1000.0 500.0] [0.0 500.0]])
(def mgr (fdir/make-fault-manager geofence 3000.0 3.0 120.0))

# Point inside geofence at [500, 250]
(h/assert-true (fdir/is-point-inside-polygon-2d [500.0 250.0] geofence) "Center point is inside geofence")
(h/assert-false (fdir/is-point-inside-polygon-2d [1500.0 250.0] geofence) "Point outside geofence boundary is rejected")

# Test Case 1: Nominal vehicle state
(def state-nom (fdir/evaluate-vehicle-safety mgr [500.0 250.0 1500.0] 15.0 10.0))
(h/assert-equal :nominal state-nom "Vehicle operates in nominal state under safe conditions")

# Test Case 2: Maximum depth violation (depth = 3200m > 3000m)
(def state-deep (fdir/evaluate-vehicle-safety mgr [500.0 250.0 3200.0] 15.0 15.0))
(h/assert-equal :abort-surface state-deep "Max depth violation triggers surface abort escalation")

# Test Case 3: Hull leak critical failure
(put mgr :leak-detected true)
(def state-leak (fdir/evaluate-vehicle-safety mgr [500.0 250.0 1000.0] 15.0 20.0))
(h/assert-equal :emergency-blow state-leak "Hull leak detection triggers emergency blow escalation")

(h/summarize-tests "Fiber Scheduler, Behavior Trees, and FDIR Executive")
