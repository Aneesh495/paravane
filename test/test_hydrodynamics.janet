# test/test_hydrodynamics.janet
# Comprehensive unit test suite for 6-DOF hydrodynamics, restoring forces, and thruster allocation.

(import ./helpers :as h)
(import ../src/paravane/core/constants :as c)
(import ../src/paravane/core/math :as m)
(import ../src/paravane/hydrodynamics/fossen :as fossen)
(import ../src/paravane/hydrodynamics/vehicle_model :as vm)
(import ../src/paravane/hydrodynamics/buoyancy_engine :as vbs)
(import ../src/paravane/hydrodynamics/thrusters :as tam)

(h/reset-test-stats)

# -----------------------------------------------------------------------------
# 6-DOF Marine Craft Kinematics
# -----------------------------------------------------------------------------

# Surge velocity 1.0 m/s with 90-degree yaw heading points East in NED
(def nu-surge [1.0 0.0 0.0 0.0 0.0 0.0])
(def eta-yaw90 [0.0 0.0 10.0 0.0 0.0 c/half-pi])
(def eta-dot (fossen/nu-to-eta-dot nu-surge eta-yaw90))

(h/assert-near 0.0 (in eta-dot 0) 1e-6 "North velocity is zero for 90-deg yaw heading")
(h/assert-near 1.0 (in eta-dot 1) 1e-6 "East velocity is 1.0 m/s for 90-deg yaw heading")
(h/assert-near 0.0 (in eta-dot 2) 1e-6 "Down velocity is zero for level surge")

# -----------------------------------------------------------------------------
# Hydrostatic Restoring Forces & Metacentric Stability
# -----------------------------------------------------------------------------

(def auv (vm/make-default-survey-auv))
(def w (get auv :weight))
(def b (get auv :buoyancy))
(def rg (get auv :rg)) # [0, 0, 0.02] m (CG is 2cm below CB)
(def rb (get auv :rb))

# Test Case 1: Level attitude (phi=0, theta=0)
(def g-level (fossen/restoring-forces w b rg rb 0.0 0.0))
# Net heave restoring force is -(W - B) = B - W (positive upward net force)
(h/assert-near (- b w) (in g-level 2) 1e-6 "Vertical restoring force matches net buoyancy")
(h/assert-near 0.0 (in g-level 3) 1e-6 "Roll restoring moment is zero at level attitude")
(h/assert-near 0.0 (in g-level 4) 1e-6 "Pitch restoring moment is zero at level attitude")

# Test Case 2: 10-degree roll angle (phi = 0.1745 rad)
(def phi10 (* 10.0 c/deg-to-rad))
(def g-roll (fossen/restoring-forces w b rg rb phi10 0.0))
# CG below CB produces positive righting moment countering roll
(h/assert-true (> (in g-roll 3) 0.0) "Roll angle generates positive restoring righting moment")

# Test Case 3: 10-degree pitch angle (theta = 0.1745 rad)
(def theta10 (* 10.0 c/deg-to-rad))
(def g-pitch (fossen/restoring-forces w b rg rb 0.0 theta10))
(h/assert-true (> (in g-pitch 4) 0.0) "Pitch angle generates positive restoring righting moment")

# -----------------------------------------------------------------------------
# Hydrodynamic Damping
# -----------------------------------------------------------------------------

(def d-lin (get auv :d-linear)) # [4.0 15.0 15.0 0.5 5.0 5.0]
(def d-quad (get auv :d-quad))   # [18.0 65.0 65.0 1.5 25.0 25.0]

# At u = 2.0 m/s surge: Drag = 4.0 * 2.0 + 18.0 * (2.0^2) = 8.0 + 72.0 = 80.0 N
(def damp-nu (fossen/hydrodynamic-damping d-lin d-quad [2.0 0.0 0.0 0.0 0.0 0.0]))
(h/assert-near 80.0 (in damp-nu 0) 1e-6 "Surge hydrodynamic damping at 2 m/s is exactly 80 N")

# -----------------------------------------------------------------------------
# Variable Buoyancy System (VBS) Tests
# -----------------------------------------------------------------------------

(def vbs-engine (vbs/make-buoyancy-engine 250.0 10.0 10.0 0.15))

# Command 100 mL positive buoyancy displacement over 5 seconds (50 mL pumped)
(def vbs-pushed (vbs/command-buoyancy-delta vbs-engine 1e-4 1e6 5.0))
(h/assert-near 5e-5 (get vbs-pushed :current-vol) 1e-9 "VBS pumps 50 mL in 5 seconds at 10 mL/s")

# Net buoyancy force for 50 mL: F = rho * g * V = 1025 * 9.80665 * 5e-5 = 0.5026 N
(def f-buoy (vbs/vbs-net-buoyancy-force vbs-pushed))
(h/assert-near 0.5026 f-buoy 0.01 "VBS generates ~0.50 N upward net buoyancy force")

# Slide battery forward by 5 cm: pitch moment = 10kg * 9.80665 * 0.05m = 4.903 N*m
(def vbs-pitched (vbs/command-pitch-trim vbs-pushed 0.05 3.0))
(def m-pitch (vbs/vbs-pitch-trim-moment vbs-pitched))
(h/assert-near 4.903 m-pitch 0.01 "Battery slide generates 4.90 N*m pitching moment")

# -----------------------------------------------------------------------------
# Thruster Allocation Matrix (TAM) Tests
# -----------------------------------------------------------------------------

(def tam-sys (tam/make-thruster-system 0.60 -0.60 0.55 35.0))

# Test Case 1: Pure surge command 20 N
(def alloc-surge (tam/allocate-thrust tam-sys [20.0 0.0 0.0 0.0 0.0 0.0]))
(def wrench-surge (tam/thruster-forces-to-wrench tam-sys (get alloc-surge :thrusts)))
(h/assert-near 20.0 (in wrench-surge 0) 1e-6 "Surge allocation delivers 20 N forward thrust")
(h/assert-near 0.0 (in wrench-surge 2) 1e-6 "Pure surge produces zero heave cross-coupling")

# Test Case 2: Pure heave command 10 N down (equal thrust on bow and stern vertical thrusters)
(def alloc-heave (tam/allocate-thrust tam-sys [0.0 0.0 10.0 0.0 0.0 0.0]))
(def u-vert (get alloc-heave :thrusts))
(h/assert-near 5.0 (in u-vert 1) 1e-6 "Bow vertical thruster takes 5 N (half of 10 N)")
(h/assert-near 5.0 (in u-vert 2) 1e-6 "Stern vertical thruster takes 5 N (half of 10 N)")
(def wrench-heave (tam/thruster-forces-to-wrench tam-sys u-vert))
(h/assert-near 10.0 (in wrench-heave 2) 1e-6 "Heave allocation delivers 10 N total heave force")
(h/assert-near 0.0 (in wrench-heave 4) 1e-6 "Equal vertical thrusters deliver zero net pitch moment")

# Test Case 3: Pure pitch moment demand 6.0 N*m (bow down, stern up)
# u_bv = -6.0 / (0.6 - (-0.6)) = -6 / 1.2 = -5 N
# u_sv = +6.0 / 1.2 = +5 N
(def alloc-pitch (tam/allocate-thrust tam-sys [0.0 0.0 0.0 0.0 6.0 0.0]))
(def wrench-pitch (tam/thruster-forces-to-wrench tam-sys (get alloc-pitch :thrusts)))
(h/assert-near 6.0 (in wrench-pitch 4) 1e-6 "Differential vertical thrust delivers 6 N*m pitch moment")
(h/assert-near 0.0 (in wrench-pitch 2) 1e-6 "Pure pitch moment delivers zero net heave force")

(h/summarize-tests "6-DOF Hydrodynamics, Restoring Moments, and Thruster Allocation")
