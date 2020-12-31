# src/paravane/guidance/terrain_following.janet
# Bottom-following altitude hold controller with terrain slope feedforward and emergency flare.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Bottom-Following Controller Representation
# -----------------------------------------------------------------------------

(defn make-terrain-controller [&opt target-alt kp kd ki max-pitch-deg emergency-alt]
  "Construct bottom-following altitude hold controller."
  (default target-alt 15.0)     # 15m nominal survey altitude above seabed
  (default kp 0.05)             # Proportional gain: rad per meter of altitude error
  (default kd 0.08)             # Derivative gain: rad per (m/s) vertical rate
  (default ki 0.002)            # Integral gain
  (default max-pitch-deg 20.0)  # 20 deg max pitch limit
  (default emergency-alt 4.0)   # 4.0m emergency terrain flare altitude threshold
  @{:target-alt target-alt
    :kp kp
    :kd kd
    :ki ki
    :max-pitch (* max-pitch-deg c/deg-to-rad)
    :emergency-alt emergency-alt
    :integrated-err 0.0})

# -----------------------------------------------------------------------------
# Altitude Control Step
# -----------------------------------------------------------------------------

(defn update-altitude-hold [ctrl current-alt altitude-rate-m-s seabed-slope-rad dt]
  "Compute commanded pitch angle to maintain constant altitude above undulating seabed."
  (def target-alt (get ctrl :target-alt))
  (def emergency-alt (get ctrl :emergency-alt))
  (def max-pitch (get ctrl :max-pitch))

  # 1. Critical safety check: emergency flare if altitude drops below minimum threshold
  (if (<= current-alt emergency-alt)
    {:pitch-cmd max-pitch # Full pitch-up command
     :emergency-flare true
     :altitude-error (- current-alt target-alt)}
    (do
      # 2. Feedforward compensation: match terrain slope
      (def ff-pitch seabed-slope-rad)

      # 3. Altitude error (positive means too high, negative means too close to bottom)
      (def alt-err (- current-alt target-alt))

      # 4. Integral error accumulation with anti-windup clamping
      (def next-int (max -10.0 (min 10.0 (+ (get ctrl :integrated-err) (* alt-err dt)))))
      (put ctrl :integrated-err next-int)

      # 5. Feedback control: if too high (alt_err > 0), pitch down (negative pitch)
      #                      if too close to seabed (alt_err < 0), pitch up (positive pitch)
      (def p-term (* (- (get ctrl :kp)) alt-err))
      (def d-term (* (- (get ctrl :kd)) altitude-rate-m-s))
      (def i-term (* (- (get ctrl :ki)) next-int))

      (def total-pitch (+ ff-pitch p-term d-term i-term))
      (def clamped-pitch (max (- max-pitch) (min max-pitch total-pitch)))

      {:pitch-cmd clamped-pitch
       :emergency-flare false
       :altitude-error alt-err})))
