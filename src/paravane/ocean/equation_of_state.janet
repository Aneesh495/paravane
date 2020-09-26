# src/paravane/ocean/equation_of_state.janet
# Thermodynamic properties of seawater: UNESCO 1980 / Saunders pressure-to-depth formulation.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Saunders (1981) Hydrostatic Pressure to Depth Conversion
# -----------------------------------------------------------------------------
# Reference:
#   Saunders, P.M. (1981). "Practical conversion of pressure to depth."
#   Journal of Physical Oceanography, 11(4), 573-574.
#
# Formula:
#   z = (1 - c1) * p - c2 * p^2
#   where:
#     c1 = (5.92 + 5.25 * sin^2(phi)) * 1e-3
#     c2 = 2.21e-6
#   p: pressure in decibars (dbar)
#   phi: latitude in radians
#   z: depth in meters (m)
# -----------------------------------------------------------------------------

(defn pressure-to-depth-saunders [p-dbar lat-rad]
  "Convert hydrostatic sea pressure in decibars to depth in meters using Saunders formulation."
  (def sin-lat (math/sin lat-rad))
  (def sin2-lat (* sin-lat sin-lat))

  (def c1 (* (+ 5.92 (* 5.25 sin2-lat)) 1e-3))
  (def c2 2.21e-6)

  (- (* (- 1.0 c1) p-dbar)
     (* c2 p-dbar p-dbar)))

(defn depth-to-pressure-approx [depth-m lat-rad]
  "Approximate hydrostatic sea pressure in decibars given depth in meters."
  (def sin-lat (math/sin lat-rad))
  (def sin2-lat (* sin-lat sin-lat))
  (def c1 (* (+ 5.92 (* 5.25 sin2-lat)) 1e-3))
  # Inverting z ~ (1 - c1) * p
  (/ depth-m (- 1.0 c1)))

# -----------------------------------------------------------------------------
# UNESCO 1980 International Equation of State (IES 80) Seawater Density
# -----------------------------------------------------------------------------
# Reference:
#   Millero, F.J., Chen, C.T., Bradshaw, A., and Schleicher, K. (1980).
#   "A new high pressure equation of state for seawater." Deep Sea Research, 27A, 255-264.
# -----------------------------------------------------------------------------

(defn pure-water-density [t]
  "Density of standard pure water at 1 atm (SMOW) in kg/m^3."
  (def t2 (* t t))
  (def t3 (* t2 t))
  (def t4 (* t3 t))
  (def t5 (* t4 t))

  (+ 999.842594
     (* 6.793952e-2 t)
     (* -9.095290e-3 t2)
     (* 1.001685e-4 t3)
     (* -1.120083e-6 t4)
     (* 6.536332e-9 t5)))

(defn seawater-density-surface [t s]
  "Density of seawater at atmospheric pressure (1 atm) in kg/m^3."
  (def rho-w (pure-water-density t))
  (def t2 (* t t))
  (def t3 (* t2 t))
  (def t4 (* t3 t))

  (def a (+ 0.824493
            (* -4.0899e-3 t)
            (* 7.6438e-5 t2)
            (* -8.2467e-7 t3)
            (* 5.3875e-9 t4)))

  (def b (+ -5.72466e-3
            (* 1.0227e-4 t)
            (* -1.6546e-6 t2)))

  (def c-coeff 4.8314e-4)

  (+ rho-w
     (* a s)
     (* b (math/pow s 1.5))
     (* c-coeff s s)))

(defn secant-bulk-modulus [t s p-bar]
  "High-pressure secant bulk modulus K(S, T, P) in bars."
  (def t2 (* t t))
  (def t3 (* t2 t))
  (def t4 (* t3 t))

  # Pure water bulk modulus at 1 atm
  (def kw (+ 19652.21
             (* 148.4206 t)
             (* -2.327105 t2)
             (* 1.360477e-2 t3)
             (* -5.155288e-5 t4)))

  (def a (+ (* 54.6746 s)
            (* -0.603459 s t)
            (* 1.09987e-2 s t2)
            (* -6.1670e-5 s t3)))

  (def b (+ (* 7.944e-2 (math/pow s 1.5))
            (* 1.6483e-2 (math/pow s 1.5) t)
            (* -5.3009e-4 (math/pow s 1.5) t2)))

  (def k0 (+ kw a b))

  # Pressure dependence
  (def p p-bar)
  (def p2 (* p p))

  (def e-w (+ 3.239908
              (* 1.43713e-3 t)
              (* 1.16092e-4 t2)
              (* -5.77905e-7 t3)))

  (def f-term (+ (* 2.2838e-3 s)
                 (* -1.0981e-5 s t)
                 (* -1.6078e-6 s t2)))

  (def g-term (+ 8.50935e-5
                 (* -6.12293e-6 t)
                 (* 5.2787e-8 t2)))

  (+ k0
     (* (+ e-w f-term) p)
     (* g-term p2)))

(defn seawater-density-ies80 [t s p-bar]
  "Compute seawater density at in-situ temperature, practical salinity, and pressure (bars) using IES 80."
  (def rho0 (seawater-density-surface t s))
  (def k (secant-bulk-modulus t s p-bar))
  (/ rho0 (- 1.0 (/ p-bar k))))

(defn density-anomaly-sigma-t [t s]
  "Compute conventional density anomaly sigma_t = rho(S, T, 0) - 1000.0 kg/m^3."
  (- (seawater-density-surface t s) 1000.0))
