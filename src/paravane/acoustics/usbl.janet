# src/paravane/acoustics/usbl.janet
# Ultra-Short Baseline (USBL) acoustic positioning and phase interferometry solver.

(import ../core/constants :as c)
(import ../core/math :as m)
(import ../ocean/profile :as prof)

# -----------------------------------------------------------------------------
# USBL Transceiver Array Configuration
# -----------------------------------------------------------------------------

(defn make-usbl-transceiver [&opt baseline-spacing carrier-freq sound-speed lever-arm]
  "Construct USBL transceiver configuration on surface vessel."
  (default baseline-spacing 0.10) # 10 cm hydrophone element spacing
  (default carrier-freq 25000.0)  # 25 kHz acoustic carrier frequency
  (default sound-speed 1500.0)    # Calibration sound speed at transceiver face
  (default lever-arm [0.0 0.0 2.5]) # Transducer pole extends 2.5m below surface hull
  (def wavelength (/ sound-speed carrier-freq))
  {:d baseline-spacing
   :freq carrier-freq
   :c sound-speed
   :wavelength wavelength
   :lever-arm lever-arm})

# -----------------------------------------------------------------------------
# Phase Difference of Arrival (PDOA) to Direction Cosines
# -----------------------------------------------------------------------------

(defn phase-diff-to-direction-cosines [transceiver delta-phi-x delta-phi-y]
  "Convert hydrophone phase differences of arrival (radians) to 3D unit direction vector."
  (def d (get transceiver :d))
  (def lambda-val (get transceiver :wavelength))
  (def k (/ c/two-pi lambda-val)) # Acoustic wavenumber

  # u = sin(theta_x) = delta_phi_x / (k * d)
  (def u (max -1.0 (min 1.0 (/ delta-phi-x (* k d)))))
  (def v (max -1.0 (min 1.0 (/ delta-phi-y (* k d)))))

  (def uv2 (+ (* u u) (* v v)))
  (def w (if (< uv2 1.0) (math/sqrt (- 1.0 uv2)) 0.0))

  # Direction vector in transceiver body frame: [forward, starboard, down]
  [u v w])

# -----------------------------------------------------------------------------
# Range Calculation from Two-Way Travel Time (TWTT)
# -----------------------------------------------------------------------------

(defn twtt-to-slant-range [twtt-seconds mean-sound-speed]
  "Compute slant range in meters from two-way travel time: R = c * dt / 2."
  (* 0.5 twtt-seconds mean-sound-speed))

# -----------------------------------------------------------------------------
# 3D Vehicle Positioning with Vessel Attitude Compensation
# -----------------------------------------------------------------------------

(defn resolve-usbl-position-ned [transceiver ship-pos-ned ship-attitude-r delta-phi-x delta-phi-y twtt-seconds mean-sound-speed]
  "Compute absolute 3D position of subsea AUV in local NED frame given surface vessel state."
  (def dir-transceiver (phase-diff-to-direction-cosines transceiver delta-phi-x delta-phi-y))
  (def slant-range (twtt-to-slant-range twtt-seconds mean-sound-speed))

  # Relative vector in transceiver body frame
  (def rel-body (m/vec3-scale dir-transceiver slant-range))

  # Transceiver location in NED frame: ship_pos + R_ship * lever_arm
  (def lever-ned (m/mat3-mul-vec ship-attitude-r (get transceiver :lever-arm)))
  (def transceiver-pos-ned (m/vec3-add ship-pos-ned lever-ned))

  # Relative vector rotated to NED frame
  (def rel-ned (m/mat3-mul-vec ship-attitude-r rel-body))

  # Absolute AUV position
  (m/vec3-add transceiver-pos-ned rel-ned))

# -----------------------------------------------------------------------------
# USBL Measurement Covariance Model (3x3 Matrix in NED)
# -----------------------------------------------------------------------------

(defn usbl-covariance-ned [slant-range ship-attitude-r &opt phase-noise-rad range-noise-m]
  "Construct 3x3 error covariance matrix for USBL position fix in local NED frame."
  (default phase-noise-rad 0.05) # 0.05 rad phase jitter (~2.8 degrees)
  (default range-noise-m 0.20)   # 20 cm acoustic ranging precision

  # Cross-range error scales with slant range: sigma_cross = R * sigma_theta
  (def sigma-cross (* slant-range phase-noise-rad))
  (def var-cross (* sigma-cross sigma-cross))
  (def var-range (* range-noise-m range-noise-m))

  # Covariance in transceiver frame: [var_cross, var_cross, var_range]
  (def cov-transceiver (m/mat3-diag var-cross var-cross var-range))

  # Rotate covariance to NED: R_ned = R * Cov_body * R^T
  (def r-t (m/mat3-transpose ship-attitude-r))
  (m/mat3-mul-mat ship-attitude-r (m/mat3-mul-mat cov-transceiver r-t)))
