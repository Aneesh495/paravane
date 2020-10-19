# src/paravane/acoustics/transmission_loss.janet
# Underwater acoustic attenuation, spreading loss, Wenz ambient noise, and sonar equation SNR.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Thorp (1967) Attenuation Formula
# -----------------------------------------------------------------------------
# Reference:
#   Thorp, W.H. (1967). "Analytic description of the low-frequency attenuation coefficient."
#   JASA, 42(1), 270.
#
# Attenuation alpha in dB/km as a function of acoustic frequency f in kHz:
#   alpha(f) = 0.11 * f^2 / (1 + f^2) + 44 * f^2 / (4100 + f^2) + 2.75e-4 * f^2 + 0.003
# -----------------------------------------------------------------------------

(defn thorp-attenuation-db-per-km [freq-khz]
  "Compute acoustic seawater absorption coefficient in dB/km using Thorp formulation."
  (def f2 (* freq-khz freq-khz))
  (def term-boric (/ (* 0.11 f2) (+ 1.0 f2))) # Boric acid chemical relaxation
  (def term-mgso4 (/ (* 44.0 f2) (+ 4100.0 f2))) # Magnesium sulfate relaxation
  (def term-pure (* 2.75e-4 f2)) # Pure water viscous loss
  (def term-base 0.003)

  (+ term-boric term-mgso4 term-pure term-base))

# -----------------------------------------------------------------------------
# Geometric Spreading Loss
# -----------------------------------------------------------------------------

(defn spherical-spreading-loss [range-m]
  "Compute spherical acoustic spreading loss in deep water: TL = 20 * log10(R)."
  (if (<= range-m 1.0)
    0.0
    (* 20.0 (/ (math/log range-m) (math/log 10.0)))))

(defn cylindrical-spreading-loss [range-m duct-height-m]
  "Compute cylindrical acoustic spreading loss in trapped acoustic waveguide: TL = 10*log10(R*H)."
  (if (<= range-m duct-height-m)
    (spherical-spreading-loss range-m)
    (let [log10 (/ 1.0 (math/log 10.0))]
      (+ (* 10.0 (math/log range-m) log10)
         (* 10.0 (math/log duct-height-m) log10)))))

(defn total-transmission-loss [range-m freq-khz &opt duct-height-m]
  "Compute total acoustic transmission loss including geometric spreading and chemical absorption."
  (def spreading (if duct-height-m
                   (cylindrical-spreading-loss range-m duct-height-m)
                   (spherical-spreading-loss range-m)))
  (def alpha (thorp-attenuation-db-per-km freq-khz))
  (def absorption (* alpha (/ range-m 1000.0)))
  (+ spreading absorption))

# -----------------------------------------------------------------------------
# Wenz (1962) Ambient Ocean Noise Spectrum
# -----------------------------------------------------------------------------
# Reference:
#   Wenz, G.M. (1962). "Acoustic ambient noise in the ocean: Spectra and sources."
#   JASA, 34(12), 1936-1956.
# -----------------------------------------------------------------------------

(defn wenz-ambient-noise-level [freq-hz &opt wind-speed-kts shipping-level]
  "Estimate total ambient ocean acoustic noise level in dB re 1 uPa^2 / Hz."
  (default wind-speed-kts 15.0) # 15 knots moderate wind (Beaufort force 4)
  (default shipping-level 0.5)  # 0.0=light, 0.5=moderate, 1.0=heavy commercial shipping

  (def f (max 1.0 freq-hz))
  (def log10-f (/ (math/log f) (math/log 10.0)))

  # Oceanic turbulence noise (dominant below 10 Hz)
  (def nl-turb (max 0.0 (- 107.0 (* 30.0 log10-f))))

  # Commercial shipping noise (dominant 10 to 500 Hz)
  (def ship-base (+ 60.0 (* 15.0 shipping-level)))
  (def nl-ship (if (and (>= f 10.0) (<= f 1000.0))
                 (max 0.0 (- ship-base (* 20.0 (math/abs (- log10-f 1.7)))))
                 0.0))

  # Wind and surface wave agitation noise (dominant 500 Hz to 50 kHz)
  (def nl-wind (max 0.0 (- (+ 50.0 (* 7.5 (math/sqrt wind-speed-kts)) (* 20.0 (/ (math/log (max 1.0 wind-speed-kts)) (math/log 10.0))))
                           (* 17.0 log10-f))))

  # Thermal molecular agitation noise (dominant above 50 kHz)
  (def nl-thermal (max 0.0 (+ -15.0 (* 20.0 log10-f))))

  # Incoherent power sum in dB: 10 * log10(sum(10^(NL_i / 10)))
  (def p-turb (math/pow 10.0 (* 0.1 nl-turb)))
  (def p-ship (math/pow 10.0 (* 0.1 nl-ship)))
  (def p-wind (math/pow 10.0 (* 0.1 nl-wind)))
  (def p-therm (math/pow 10.0 (* 0.1 nl-thermal)))

  (def p-total (+ p-turb p-ship p-wind p-therm))
  (* 10.0 (/ (math/log p-total) (math/log 10.0))))

# -----------------------------------------------------------------------------
# Sonar Equation Signal-to-Noise Ratio (SNR)
# -----------------------------------------------------------------------------

(defn sonar-snr-passive [source-level range-m freq-khz noise-level directivity-index]
  "Compute passive sonar equation SNR in dB: SNR = SL - TL - NL + DI."
  (def tl (total-transmission-loss range-m freq-khz))
  (+ source-level (- tl) (- noise-level) directivity-index))

(defn sonar-snr-active [source-level target-strength range-m freq-khz noise-level directivity-index]
  "Compute active sonar equation SNR in dB: SNR = SL - 2*TL + TS - NL + DI."
  (def tl (total-transmission-loss range-m freq-khz))
  (+ source-level (* -2.0 tl) target-strength (- noise-level) directivity-index))
