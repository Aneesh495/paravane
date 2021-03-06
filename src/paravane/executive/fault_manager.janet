# src/paravane/executive/fault_manager.janet
# Fault Detection, Isolation, and Recovery (FDIR), polygonal geofencing, and abort executive.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Polygonal Geofencing (Even-Odd Ray Casting Algorithm)
# -----------------------------------------------------------------------------

(defn is-point-inside-polygon-2d [point-xy polygon-vertices]
  "Determine whether 2D point [x, y] lies inside polygonal boundary using ray casting."
  (def px (in point-xy 0))
  (def py (in point-xy 1))
  (def n (length polygon-vertices))
  (var inside false)

  (var j (- n 1))
  (for i 0 n
    (def xi (in (in polygon-vertices i) 0))
    (def yi (in (in polygon-vertices i) 1))
    (def xj (in (in polygon-vertices j) 0))
    (def yj (in (in polygon-vertices j) 1))

    (def intersect (and (not= (> yi py) (> yj py))
                        (< px (+ (/ (* (- xj xi) (- py yi)) (- yj yi)) xi))))
    (when intersect
      (set inside (not inside)))
    (set j i))

  inside)

# -----------------------------------------------------------------------------
# Fault Manager Representation
# -----------------------------------------------------------------------------

(defn make-fault-manager [geofence-poly max-depth-m min-alt-m max-acoustic-silence-s]
  "Construct FDIR fault manager."
  @{:geofence geofence-poly
    :max-depth max-depth-m
    :min-alt min-alt-m
    :max-silence max-acoustic-silence-s
    :last-acoustic-time 0.0
    :leak-detected false
    :battery-soc 1.0 # 100% capacity
    :state :nominal
    :fault-log @[]})

# -----------------------------------------------------------------------------
# Subsea Health Evaluation Cycle
# -----------------------------------------------------------------------------

(defn evaluate-vehicle-safety [fdir pos-ned altitude-m current-time-s]
  "Evaluate vehicle sensor health, geofence, and depth boundaries. Returns escalation state."
  (def x (in pos-ned 0))
  (def y (in pos-ned 1))
  (def depth (in pos-ned 2))

  (cond
    # 1. Critical Hull Leak: immediate emergency ascent
    (get fdir :leak-detected)
    (do
      (put fdir :state :emergency-blow)
      (array/push (get fdir :fault-log) "CRITICAL: Hull water ingress detected")
      :emergency-blow)

    # 2. Critical Battery Depletion (< 15%)
    (< (get fdir :battery-soc) 0.15)
    (do
      (put fdir :state :abort-surface)
      (array/push (get fdir :fault-log) "WARNING: Low battery capacity")
      :abort-surface)

    # 3. Maximum Depth Violation
    (> depth (get fdir :max-depth))
    (do
      (put fdir :state :abort-surface)
      (array/push (get fdir :fault-log) "VIOLATION: Vehicle exceeded maximum safe depth")
      :abort-surface)

    # 4. Geofence Boundary Breach
    (not (is-point-inside-polygon-2d [x y] (get fdir :geofence)))
    (do
      (put fdir :state :abort-surface)
      (array/push (get fdir :fault-log) "VIOLATION: Vehicle breached operational geofence")
      :abort-surface)

    # 5. Acoustic Comms Watchdog Timeout
    (> (- current-time-s (get fdir :last-acoustic-time)) (get fdir :max-silence))
    (do
      (put fdir :state :degraded)
      :degraded)

    # Nominal operation
    (do
      (put fdir :state :nominal)
      :nominal)))
