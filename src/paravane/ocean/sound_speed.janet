# src/paravane/ocean/sound_speed.janet
# High-precision empirical and thermodynamic sound speed algorithms in seawater.

(import ../core/constants :as c)

# -----------------------------------------------------------------------------
# Mackenzie (1981) Nine-Term Sound Speed Equation
# -----------------------------------------------------------------------------
# References:
#   Mackenzie, K.V. (1981). "Nine-term equation for sound speed in the oceans."
#   Journal of the Acoustical Society of America, 70(3), 807-812.
#
# Inputs:
#   t : Temperature in degrees Celsius (valid 0.0 to 30.0 C)
#   s : Practical salinity in PSU (valid 25.0 to 40.0 PSU)
#   d : Subsea depth in meters (valid 0.0 to 8000.0 m)
# Output:
#   Speed of sound in meters per second (RMS standard deviation: 0.070 m/s)
# -----------------------------------------------------------------------------

(defn sound-speed-mackenzie [t s d]
  "Compute sound speed in seawater using Mackenzie (1981) nine-term formulation."
  (def t2 (* t t))
  (def t3 (* t2 t))
  (def d2 (* d d))
  (def d3 (* d2 d))
  (def s-35 (- s 35.0))

  (+ 1448.96
     (* 4.591 t)
     (* -5.304e-2 t2)
     (* 2.374e-4 t3)
     (* 1.340 s-35)
     (* 1.630e-2 d)
     (* 1.675e-7 d2)
     (* -1.025e-2 t s-35)
     (* -7.139e-13 t d3)))

# -----------------------------------------------------------------------------
# UNESCO Chen and Millero (1977) / Wong and Zhu (1995) International Standard
# -----------------------------------------------------------------------------
# Reference:
#   Wong, G.S.K., and Zhu, S. (1995). "Speed of sound in seawater as a function
#   of temperature, salinity, and pressure." JASA, 97(3), 1732-1736.
# -----------------------------------------------------------------------------

(defn sound-speed-unesco [t s p-bar]
  "Compute sound speed using UNESCO Chen-Millero formulation with pressure in bars."
  (def t2 (* t t))
  (def t3 (* t2 t))
  (def t4 (* t3 t))
  (def t5 (* t4 t))

  (def p p-bar)
  (def p2 (* p p))
  (def p3 (* p2 p))

  # Cw: Pure water sound speed polynomial with temperature and pressure
  (def cw0 (+ 1402.388
              (* 5.03830 t)
              (* -5.81090e-2 t2)
              (* 3.3432e-4 t3)
              (* -1.47797e-6 t4)
              (* 3.1419e-9 t5)))

  (def cw1 (+ 0.153563
              (* 6.8999e-4 t)
              (* -8.1829e-6 t2)
              (* 1.3632e-7 t3)
              (* -6.1260e-10 t4)))

  (def cw2 (+ 3.1260e-5
              (* -1.7111e-6 t)
              (* 2.5986e-8 t2)
              (* -2.5353e-10 t3)
              (* 1.0415e-12 t4)))

  (def cw3 (+ -9.7729e-9
              (* 3.8513e-10 t)
              (* -2.3654e-12 t2)))

  (def cw (+ cw0 (* cw1 p) (* cw2 p2) (* cw3 p3)))

  # A: Salinity linear term with temperature and pressure dependence
  (def a0 (+ 1.389
             (* -1.262e-2 t)
             (* 7.166e-5 t2)
             (* 2.008e-6 t3)
             (* -3.21e-8 t4)))

  (def a1 (+ 9.4742e-5
             (* -1.2583e-5 t)
             (* -6.4928e-8 t2)
             (* 1.0515e-8 t3)
             (* -2.0142e-10 t4)))

  (def a2 (+ -3.9064e-7
             (* 9.1061e-9 t)
             (* -1.6009e-10 t2)
             (* 7.994e-12 t3)))

  (def a3 (+ 1.100e-10
             (* 6.651e-12 t)
             (* -3.391e-13 t2)))

  (def a (+ a0 (* a1 p) (* a2 p2) (* a3 p3)))

  # B: Salinity 3/2 power term
  (def b0 (+ -1.922e-2 (* -4.42e-5 t)))
  (def b1 (+ 7.3637e-5 (* 1.7950e-7 t)))
  (def b (+ b0 (* b1 p)))

  # D: Salinity squared term
  (def d (+ 1.727e-3 (* -7.9836e-6 p)))

  (+ cw
     (* a s)
     (* b (math/pow s 1.5))
     (* d s s)))

# -----------------------------------------------------------------------------
# Del Grosso (1974) Formulation (NRL standard)
# -----------------------------------------------------------------------------

(defn sound-speed-del-grosso [t s p-kg-cm2]
  "Compute sound speed in seawater using Del Grosso (1974) NRL formulation."
  # Pure water sound speed at 0 kg/cm^2 gauge pressure
  (def c000 1402.392)
  (def dct (+ (* 5.01109398873 t)
              (* -5.50946843172e-2 (* t t))
              (* 2.21535969240e-4 (* t t t))
              (* -5.84976127260e-7 (* t t t t))))

  (def dcs (+ (* 1.32952290781 s)
              (* 1.28955756844e-4 (* s s))))

  (def dcp (+ (* 0.156059280041 p-kg-cm2)
              (* 2.44998688441e-5 (* p-kg-cm2 p-kg-cm2))
              (* -8.83392332513e-9 (* p-kg-cm2 p-kg-cm2 p-kg-cm2))))

  (def cross (+ (* -1.27562783426e-2 t s)
                (* 6.35191613389e-3 t p-kg-cm2)
                (* 2.65484716608e-7 (* t t) (* p-kg-cm2 p-kg-cm2))
                (* -1.59349479045e-5 (* t t) p-kg-cm2)
                (* 5.22116437235e-9 t (* p-kg-cm2 p-kg-cm2 p-kg-cm2))
                (* -4.38029728046e-6 (* t t t) p-kg-cm2)
                (* -1.61674495909e-8 (* s s) (* p-kg-cm2 p-kg-cm2))
                (* 9.68403156410e-5 t s p-kg-cm2)))

  # Del Grosso formulation at surface is authoritative; at depth UNESCO is international standard
  (+ c000 dct dcs dcp cross))

# -----------------------------------------------------------------------------
# Coppens (1981) Simple Formula for Shallow Waters
# -----------------------------------------------------------------------------

(defn sound-speed-coppens [t s d]
  "Simple sound speed approximation in shallow water (depth < 4000m)."
  (def t-norm (/ t 10.0))
  (def d-km (/ d 1000.0))
  (def c0 1449.05)
  (def t-term (+ (* 45.7 t-norm)
                 (* -5.21 (* t-norm t-norm))
                 (* 0.23 (* t-norm t-norm t-norm))))
  (def s-term (+ (* (- s 35.0) (- 1.333 (* 0.126 t-norm)))
                 (* (math/pow (- s 35.0) 2) (- 0.009 (* 0.0009 t-norm)))))
  (def d-term (+ (* 16.3 d-km) (* 0.18 (* d-km d-km))))
  (+ c0 t-term s-term d-term))

(defn sound-speed-gradient [t s d &opt delta-z]
  "Compute vertical sound speed gradient dc/dz in (m/s) per meter at specified depth."
  (default delta-z 1.0)
  (def c-plus (sound-speed-mackenzie t s (+ d delta-z)))
  (def c-minus (sound-speed-mackenzie t s (- d delta-z)))
  (/ (- c-plus c-minus) (* 2.0 delta-z)))
