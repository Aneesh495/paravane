# src/paravane/hydrodynamics/buoyancy_engine.janet
# Variable Buoyancy System (VBS) hydraulic pump and internal sliding mass pitch trim.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Variable Buoyancy System Representation
# -----------------------------------------------------------------------------

(defn make-buoyancy-engine [&opt max-oil-vol-ml pump-rate-ml-s battery-mass-kg max-slide-m]
  "Construct Variable Buoyancy System (VBS) model."
  (default max-oil-vol-ml 250.0)   # +/- 250 mL oil displacement
  (default pump-rate-ml-s 10.0)    # 10 mL/s hydraulic pump capacity
  (default battery-mass-kg 10.0)   # 10 kg internal sliding mass
  (default max-slide-m 0.15)       # +/- 15 cm longitudinal travel
  @{:max-vol (/ max-oil-vol-ml 1e6) # Convert mL to m^3
    :pump-rate (/ pump-rate-ml-s 1e6)
    :current-vol 0.0 # 0.0 = neutral trim
    :bat-mass battery-mass-kg
    :max-slide max-slide-m
    :current-slide 0.0
    :total-energy-joules 0.0})

# -----------------------------------------------------------------------------
# Bladder Pumping Dynamics
# -----------------------------------------------------------------------------

(defn command-buoyancy-delta [vbs target-vol-m3 ambient-pressure-pa dt]
  "Transfer hydraulic fluid to/from external bladder against ambient hydrostatic pressure."
  (def max-v (get vbs :max-vol))
  (def clamped-target (max (- max-v) (min max-v target-vol-m3)))
  (def cur-v (get vbs :current-vol))
  (def dv-desired (- clamped-target cur-v))

  (def max-dv (* (get vbs :pump-rate) dt))
  (def dv-actual (max (- max-dv) (min max-dv dv-desired)))
  (def next-v (+ cur-v dv-actual))

  # Mechanical work done by pump against seawater pressure: W = P * dV
  (def work-done (if (> dv-actual 0.0) (* ambient-pressure-pa dv-actual) 0.0))
  (def next-energy (+ (get vbs :total-energy-joules) work-done))

  (def next-vbs (table/clone vbs))
  (put next-vbs :current-vol next-v)
  (put next-vbs :total-energy-joules next-energy)
  next-vbs)

(defn vbs-net-buoyancy-force [vbs &opt seawater-density]
  "Compute additional vertical buoyancy force produced by external bladder volume in Newtons."
  (default seawater-density c/default-seawater-density)
  (* (get vbs :current-vol) seawater-density c/standard-gravity))

# -----------------------------------------------------------------------------
# Pitch Trimming via Sliding Mass
# -----------------------------------------------------------------------------

(defn command-pitch-trim [vbs target-slide-m dt]
  "Move internal battery mass longitudinally to adjust center of gravity and vehicle pitch angle."
  (def max-s (get vbs :max-slide))
  (def clamped-target (max (- max-s) (min max-s target-slide-m)))
  (def cur-s (get vbs :current-slide))
  (def ds-desired (- clamped-target cur-s))

  # Actuator speed limit: 2 cm/s
  (def max-ds (* 0.02 dt))
  (def ds-actual (max (- max-ds) (min max-ds ds-desired)))
  (def next-s (+ cur-s ds-actual))

  (def next-vbs (table/clone vbs))
  (put next-vbs :current-slide next-s)
  next-vbs)

(defn vbs-pitch-trim-moment [vbs]
  "Compute longitudinal gravity pitching moment in N*m produced by shifted battery mass."
  # M_pitch = m_bat * g * delta_x
  (* (get vbs :bat-mass) c/standard-gravity (get vbs :current-slide)))
